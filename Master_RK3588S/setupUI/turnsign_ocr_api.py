import json
import multiprocessing as mp
import os
import queue
import re
import threading
import time
from collections import deque
from difflib import SequenceMatcher

import cv2
import numpy as np


DEFAULT_API_TYPE = "aistudio-v3"
DEFAULT_API_BASE_URL = "https://aistudio.baidu.com/llm/lmapi/v3"
DEFAULT_MODEL = "ernie-4.5-turbo-32k"
API_CACHE_VERSION = os.environ.get("AR_TURNSIGN_API_CACHE_VERSION", "api_v3_left_right")

DEFAULT_SYSTEM_PROMPT = """你是智能车路牌语义决策器。赛道在每个决策点只有左、右两条实际道路，不存在第三条可选道路。

请理解 OCR 原文表达的完整语义，而不是依靠关键词匹配、固定句式、示例套用或字面顺序。文字可能包含口语、委婉表达、否定、双重否定、转折、因果、程度差异、多个方案的相对比较、反问、反讽、句末语气反转、道路状态、绕行要求或不完整识别结果。

请在内部完成以下语义推理，但不要输出推理过程：
- 识别句子实际谈到的两条道路，以及同义称呼、临时描述、指代词和省略成分分别指向哪一条道路。
- 当一句话把一条已明确左/右的道路与另一条用形态、位置或特征描述的道路进行对比时，它们是两个不同选项；由于现场只有两条路，另一个选项对应相反的物理侧。
- 还原否定、双重否定、反问、反讽和句末语气反转之后的真实道路状态，不能看到局部否定词就直接下结论。
- 比较两条道路的通行严重程度。“不便、难走、较差”和“封闭、完全不通、无法通过”不是同一等级；应选择实际仍能通行且整体更可行的道路。
- 综合整句话作出最终选择，不能只根据某一个词决定方向。

“直道、直行、前方”等词描述的可能是道路形态或相对关系，不是第三种输出方向。它在当前场景中可能对应左侧道路，也可能对应右侧道路；必须根据整段文字表达的通行与绕行关系进行理解，不能固定映射到某一侧。

只输出最终物理方向，不输出分析过程。只有在 OCR 信息足以作出明确判断时才输出 left 或 right；信息不足、文字残缺、语义冲突或无法可靠判断时必须输出 unknown，禁止猜测或默认选择任意一侧。

只能输出以下三个 JSON 之一，不要 Markdown，不要原因，不要任何其他字段或文字：
{"direction":"left"}
{"direction":"right"}
{"direction":"unknown"}
"""


def now_seconds():
    return time.time()


def load_local_ocr_env(base_dir=None):
    """Load local OCR/API env values without executing shell code."""
    if os.environ.get("AISTUDIO_ACCESS_TOKEN") or os.environ.get("EB_ACCESS_TOKEN"):
        return False
    base_dir = base_dir or os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(base_dir, "ocr_env.sh")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                match = re.match(r"\s*export\s+([A-Z0-9_]*ACCESS_TOKEN)\s*=\s*(.*?)\s*$", raw_line)
                if not match:
                    continue
                key = match.group(1)
                value = match.group(2).strip().strip("\"'")
                if value and "your-token" not in value and "你的token" not in value:
                    os.environ[key] = value
                    if key != "AISTUDIO_ACCESS_TOKEN":
                        os.environ.setdefault("AISTUDIO_ACCESS_TOKEN", value)
                    return True
    except FileNotFoundError:
        return False
    except OSError:
        return False
    return False


def repair_python_logging_levels():
    """Repair stdlib logging level-name maps after native runtimes corrupt them.

    RKNNLite imports on this board can clear entries such as "INFO" from
    logging._nameToLevel.  Paddle imports later call logger.setLevel("INFO") and
    then fail with "Unknown level: 'INFO'".  Re-registering the standard names is
    safe and idempotent.
    """
    try:
        import logging

        for name, value in (
            ("CRITICAL", logging.CRITICAL),
            ("FATAL", logging.FATAL),
            ("ERROR", logging.ERROR),
            ("WARN", logging.WARNING),
            ("WARNING", logging.WARNING),
            ("INFO", logging.INFO),
            ("DEBUG", logging.DEBUG),
            ("NOTSET", logging.NOTSET),
        ):
            logging._nameToLevel[name] = int(value)
            logging._levelToName[int(value)] = name if name != "WARN" else "WARNING"
            logging.addLevelName(int(value), name)
        return True
    except Exception:
        return False


def limit_ocr_cpu_threads():
    """Keep PaddleOCR from spawning enough CPU workers to starve the RK board."""
    thread_count = str(max(1, int(os.environ.get("AR_PADDLEOCR_CPU_THREADS", "1"))))
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
    ):
        os.environ.setdefault(key, thread_count)
    os.environ.setdefault("FLAGS_use_mkldnn", "0")
    os.environ.setdefault("FLAGS_enable_mkldnn", "0")
    os.environ.setdefault("FLAGS_cpu_math_library_num_threads", thread_count)
    return int(thread_count)


def clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def normalize_text(text):
    text = str(text or "").strip()
    text = re.sub(r"\s+", "", text)
    text = text.replace("：", ":").replace("，", ",").replace("。", ".")
    text = text.replace("；", ";").replace("（", "(").replace("）", ")")
    return text


def safe_json_loads(text):
    if not text:
        return None
    text = str(text).strip()
    text = re.sub(r"^```(?:json)?\s*", "", text, flags=re.I)
    text = re.sub(r"\s*```$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    match = re.search(r"\{.*\}", text, flags=re.S)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError:
        return None


def parse_direction_payload(text):
    payload = safe_json_loads(text)
    if isinstance(payload, dict):
        return payload

    raw = str(payload if isinstance(payload, str) else text or "").strip().lower()
    raw = re.sub(r"^```(?:json)?\s*", "", raw, flags=re.I)
    raw = re.sub(r"\s*```$", "", raw)
    raw = raw.strip(" \t\r\n\"'`。.")
    aliases = {
        "left": "left",
        "左": "left",
        "左边": "left",
        "左侧": "left",
        "right": "right",
        "右": "right",
        "右边": "right",
        "右侧": "right",
        "unknown": "unknown",
        "未知": "unknown",
        "无法判断": "unknown",
    }
    direction = aliases.get(raw)
    return {"direction": direction} if direction else None


def fixed_unknown_result(source_text="", reason="unknown"):
    return {
        "valid": False,
        "instruction_type": "unknown",
        "action": "unknown",
        "preferred_branch": "unknown",
        "direction": "unknown",
        "avoid_branches": [],
        "branch_status": {
            "left": "unknown",
            "right": "unknown",
        },
        "execute_timing": "unknown",
        "confidence": 0.0,
        "reason": str(reason),
        "source_text": str(source_text or ""),
    }


def normalize_instruction(payload, source_text=""):
    if not isinstance(payload, dict):
        return fixed_unknown_result(source_text, "api_invalid_json")
    direction = str(payload.get("direction") or payload.get("preferred_branch") or "").strip().lower()
    if direction not in {"left", "right"}:
        return fixed_unknown_result(source_text, "api_invalid_direction")

    result = fixed_unknown_result(source_text, "api_direction")
    result.update(
        {
            "valid": True,
            "instruction_type": "branch_choice",
            "action": "choose_branch",
            "preferred_branch": direction,
            "direction": direction,
            "confidence": 1.0,
            "reason": "api_direction",
            "source_text": str(source_text or "")[:300],
        }
    )
    return result


def crop_bbox(frame, bbox, pad_ratio=0.12):
    if frame is None or bbox is None:
        return None
    h, w = frame.shape[:2]
    left, top, right, bottom = [float(v) for v in bbox]
    bw = max(1.0, right - left)
    bh = max(1.0, bottom - top)
    pad_x = bw * float(pad_ratio)
    pad_y = bh * float(pad_ratio)
    x0 = int(clamp(left - pad_x, 0, w - 1))
    y0 = int(clamp(top - pad_y, 0, h - 1))
    x1 = int(clamp(right + pad_x, x0 + 1, w))
    y1 = int(clamp(bottom + pad_y, y0 + 1, h))
    return frame[y0:y1, x0:x1].copy()


def image_ahash(image, size=16):
    if image is None or image.size == 0:
        return ""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    small = cv2.resize(gray, (size, size), interpolation=cv2.INTER_AREA)
    avg = float(np.mean(small))
    bits = (small > avg).astype(np.uint8).reshape(-1)
    value = 0
    chars = []
    for idx, bit in enumerate(bits):
        value = (value << 1) | int(bit)
        if idx % 4 == 3:
            chars.append(format(value, "x"))
            value = 0
    return "".join(chars)


class PaddleOcrReader:
    def __init__(self, lang="ch", use_angle_cls=True, model_root=None):
        cpu_threads = limit_ocr_cpu_threads()
        repair_python_logging_levels()
        try:
            from paddleocr import PaddleOCR
        except Exception as exc:
            raise RuntimeError(f"PaddleOCR import failed: {exc}") from exc

        model_root = model_root or os.environ.get("AR_PADDLEOCR_MODEL_ROOT") or os.environ.get("PADDLEOCR_MODEL_ROOT")
        kwargs = {
            "lang": lang,
            "use_angle_cls": bool(use_angle_cls),
            "use_gpu": False,
            "enable_mkldnn": False,
            "cpu_threads": cpu_threads,
        }
        if model_root:
            kwargs.update(self._model_dirs(model_root))
        try:
            self.ocr_engine = PaddleOCR(show_log=False, **kwargs)
        except TypeError:
            self.ocr_engine = PaddleOCR(**kwargs)

    @staticmethod
    def _model_dirs(model_root):
        return {
            "det_model_dir": os.path.join(model_root, "ch_PP-OCRv4_det_infer"),
            "rec_model_dir": os.path.join(model_root, "ch_PP-OCRv4_rec_infer"),
            "cls_model_dir": os.path.join(model_root, "ch_ppocr_mobile_v2.0_cls_infer"),
        }

    def read(self, image):
        if image is None or image.size == 0:
            return {"text": "", "confidence": 0.0, "lines": []}

        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image.ndim == 3 else image
        try:
            raw = self.ocr_engine.ocr(rgb, cls=True)
        except TypeError:
            raw = self.ocr_engine.ocr(rgb)
        return self._parse_ocr_result(raw)

    @staticmethod
    def _parse_ocr_result(raw):
        lines = []

        def add_line(text, score):
            text = normalize_text(text)
            if not text:
                return
            try:
                score = float(score)
            except Exception:
                score = 0.0
            lines.append({"text": text, "confidence": score})

        def parse_item(item):
            if isinstance(item, dict):
                if "rec_texts" in item:
                    scores = item.get("rec_scores") or []
                    for idx, text in enumerate(item.get("rec_texts") or []):
                        score = scores[idx] if idx < len(scores) else 0.0
                        add_line(text, score)
                    return
                text = item.get("text") or item.get("rec_text") or item.get("transcription")
                score = item.get("confidence") or item.get("score") or item.get("rec_score") or 0.0
                add_line(text, score)
                return
            if isinstance(item, (list, tuple)):
                if (
                    len(item) == 2
                    and isinstance(item[1], (list, tuple))
                    and len(item[1]) >= 2
                    and isinstance(item[1][0], str)
                ):
                    add_line(item[1][0], item[1][1])
                else:
                    for child in item:
                        parse_item(child)

        parse_item(raw)
        text = "".join(line["text"] for line in lines)
        confidence = 0.0
        if lines:
            confidence = float(sum(line["confidence"] for line in lines) / len(lines))
        return {"text": text, "confidence": confidence, "lines": lines}


class WenxinRouteInterpreter:
    def __init__(
        self,
        access_token=None,
        model=None,
        timeout=8.0,
        temperature=0.1,
        system_prompt=None,
        log_func=None,
        api_key=None,
        api_url=None,
    ):
        # api_key remains supported for old callers; both names contain the
        # AI Studio access token used by its OpenAI-compatible endpoint.
        load_local_ocr_env()
        self.access_token = (
            access_token
            or api_key
            or os.environ.get("AR_TURNSIGN_ACCESS_TOKEN")
            or os.environ.get("AISTUDIO_ACCESS_TOKEN")
            or os.environ.get("EB_ACCESS_TOKEN")
            or os.environ.get("AI_STUDIO_ACCESS_TOKEN")
            or os.environ.get("AI_STUDIO_API_KEY")
        )
        self.model = model or os.environ.get("AR_TURNSIGN_MODEL") or os.environ.get("EB_MODEL") or DEFAULT_MODEL
        self.timeout = max(0.1, float(os.environ.get("AR_TURNSIGN_API_TIMEOUT", timeout)))
        self.api_enabled = os.environ.get("AR_TURNSIGN_API_ENABLED", "1").strip().lower() not in {
            "0",
            "false",
            "no",
            "off",
        }
        # AI Studio currently rejects 0.0; keep the output deterministic while
        # remaining inside its accepted interval (0, 1].
        self.temperature = max(0.01, min(1.0, float(temperature)))
        self.system_prompt = system_prompt or os.environ.get("AR_TURNSIGN_PROMPT") or DEFAULT_SYSTEM_PROMPT
        self.api_type = DEFAULT_API_TYPE
        self.api_url = (
            api_url
            or os.environ.get("AR_TURNSIGN_API_BASE_URL")
            or DEFAULT_API_BASE_URL
        ).rstrip("/")
        self.max_retries = max(0, int(os.environ.get("AR_TURNSIGN_API_MAX_RETRIES", "2")))
        self.trust_env = os.environ.get("AR_TURNSIGN_API_TRUST_ENV", "0").strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        self._api_client = None
        self._http_client = None
        self.log_func = log_func or (lambda message: None)

    def enabled(self):
        return self.api_enabled and bool(self.access_token)

    def interpret(self, ocr_text, ocr_confidence=None, context=None):
        source_text = normalize_text(ocr_text)
        if not source_text:
            return fixed_unknown_result("", "empty_ocr_text")
        if not self.enabled():
            reason = "api_disabled" if not self.api_enabled else "missing_access_token"
            result = fixed_unknown_result(source_text, reason)
            result["ocr_confidence"] = float(ocr_confidence or 0.0)
            result["api_type"] = self.api_type
            result["api_model"] = self.model
            result["api_error"] = reason
            return result

        user_payload = self._build_user_payload(source_text, ocr_confidence, context)
        try:
            content = self._call_openai(user_payload)
        except ImportError as exc:
            result = fixed_unknown_result(source_text, "openai_import_failed")
            result["api_error"] = str(exc)
            result["install_hint"] = "pip install openai httpx"
            result["api_type"] = self.api_type
            result["api_model"] = self.model
            result["ocr_confidence"] = float(ocr_confidence or 0.0)
            return result
        except Exception as exc:
            self.log_func(f"turnsign AI Studio v3 call failed: {exc}")
            result = fixed_unknown_result(source_text, "api_request_failed")
            result["api_error"] = str(exc)
            result["api_type"] = self.api_type
            result["api_model"] = self.model
            result["ocr_confidence"] = float(ocr_confidence or 0.0)
            return result

        payload = parse_direction_payload(content)
        result = normalize_instruction(payload, source_text=source_text)
        result["ocr_confidence"] = float(ocr_confidence or 0.0)
        result["api_type"] = self.api_type
        result["api_model"] = self.model
        result["api_raw_content"] = str(content or "")[:1000]
        if isinstance(payload, dict):
            result["api_payload"] = payload
        if payload is None:
            result["api_parse_error"] = "api_invalid_json"
        return result

    def _ensure_api_client(self):
        if self._api_client is not None:
            return self._api_client
        try:
            import httpx
            from openai import OpenAI
        except Exception as exc:
            raise ImportError(f"OpenAI-compatible client import failed: {exc}") from exc

        self._http_client = httpx.Client(
            trust_env=self.trust_env,
            timeout=self.timeout,
        )
        self._api_client = OpenAI(
            api_key=self.access_token,
            base_url=self.api_url,
            timeout=self.timeout,
            max_retries=self.max_retries,
            http_client=self._http_client,
        )
        return self._api_client

    def _call_openai(self, user_payload):
        client = self._ensure_api_client()

        content = (
            "请分析以下 OCR 数据并判断最终方向：\n"
            f"{user_payload}\n"
            "只能输出 {\"direction\":\"left\"}、{\"direction\":\"right\"} "
            "或 {\"direction\":\"unknown\"}。"
        )
        response = client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": content},
            ],
            temperature=self.temperature,
        )
        return self._extract_content(response)

    @staticmethod
    def _build_user_payload(ocr_text, ocr_confidence=None, context=None):
        payload = {
            "ocr_text": ocr_text,
            "ocr_confidence": float(ocr_confidence or 0.0),
            "context": context or {},
            "required_output": {
                "format": "json_object_only",
                "schema": {"direction": "left_right_or_unknown"},
            },
        }
        return json.dumps(payload, ensure_ascii=False)

    @staticmethod
    def _extract_content(response):
        if response is None:
            return ""
        if isinstance(response, str):
            return response
        if hasattr(response, "get_result"):
            try:
                result = response.get_result()
                if result:
                    return result
            except Exception:
                pass
        if hasattr(response, "to_dict"):
            try:
                response = response.to_dict()
            except Exception:
                pass
        if isinstance(response, dict):
            for key in ("result", "content", "text"):
                if response.get(key):
                    return response.get(key)
            choices = response.get("choices") or []
            if choices:
                message = choices[0].get("message") or {}
                if isinstance(message, dict):
                    return message.get("content") or ""
                return str(message)
        for attr in ("result", "content", "text"):
            if hasattr(response, attr):
                value = getattr(response, attr)
                if value:
                    return value
        return str(response)


class TurnSignOcrApiProcessor:
    def __init__(
        self,
        min_det_score=0.40,
        min_area_ratio=0.031,
        min_ocr_confidence=0.40,
        stable_frames=2,
        stable_duration_s=0.50,
        stable_bypass_confidence=0.90,
        stable_bypass_min_text_len=6,
        text_similarity=0.86,
        ocr_interval=0.25,
        api_cooldown=2.0,
        cache_ttl=30.0,
        async_api=True,
        ocr_reader=None,
        interpreter=None,
        log_func=None,
    ):
        self.min_det_score = float(min_det_score)
        self.min_area_ratio = float(min_area_ratio)
        self.min_ocr_confidence = float(min_ocr_confidence)
        self.stable_frames = int(stable_frames)
        self.stable_duration_s = max(
            0.0,
            float(os.environ.get("AR_TURNSIGN_STABLE_DURATION_S", stable_duration_s)),
        )
        self.stable_bypass_confidence = float(
            os.environ.get("AR_TURNSIGN_STABLE_BYPASS_CONFIDENCE", stable_bypass_confidence)
        )
        self.stable_bypass_min_text_len = max(
            1,
            int(os.environ.get("AR_TURNSIGN_STABLE_BYPASS_MIN_TEXT_LEN", stable_bypass_min_text_len)),
        )
        self.text_similarity = float(text_similarity)
        self.ocr_interval = float(ocr_interval)
        self.api_cooldown = float(api_cooldown)
        self.cache_ttl = float(cache_ttl)
        self.async_api = bool(async_api)
        self.log_func = log_func or (lambda message: None)

        self.ocr_reader = ocr_reader
        self.ocr_init_error = None
        self.last_ocr_init_attempt_ts = 0.0
        self.ocr_init_retry_interval = 5.0
        self.interpreter = interpreter or WenxinRouteInterpreter(log_func=self.log_func)

        self.last_ocr_ts = 0.0
        self.last_api_ts = 0.0
        self.last_text = ""
        self.stable_since_ts = 0.0
        self.stable_count = 0
        self.history = deque(maxlen=8)
        self.cache = {}
        self.pending_thread = None
        self.pending_text = ""
        self.last_api_text = ""
        self.last_api_result_ts = 0.0
        self.api_result_hold = float(os.environ.get("AR_TURNSIGN_API_RESULT_HOLD", "15.0"))
        self.latest_result = fixed_unknown_result("", "not_started")

    def _ensure_ocr(self):
        if self.ocr_reader is None:
            self.ocr_reader = PaddleOcrReader()
        self.ocr_init_error = None

    @staticmethod
    def is_turnsign_detection(det):
        label = str(det.get("label") or "").strip().lower().replace("_", "").replace("-", "")
        category = str(det.get("category") or "").strip().lower().replace("_", "").replace("-", "")
        return label in {"turnsign", "turn sign"} or category in {"turnsign", "turn sign"}

    def select_turnsign(self, detections):
        candidates = []
        for det in detections or []:
            if not self.is_turnsign_detection(det):
                continue
            try:
                score = float(det.get("score", 0.0))
                area = float(det.get("area_ratio", 0.0))
            except Exception:
                continue
            if score < self.min_det_score or area < self.min_area_ratio:
                continue
            rank = score + area * 4.0
            candidates.append((rank, det))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def process(self, frame, detections, timestamp=None, context=None):
        ts = float(timestamp if timestamp is not None else now_seconds())
        det = self.select_turnsign(detections)
        if det is None:
            return {
                "active": False,
                "status": "no_turnsign",
                "instruction": fixed_unknown_result("", "no_turnsign"),
                "latest_instruction": self.latest_result,
                "instruction_current": False,
            }

        crop = crop_bbox(frame, det.get("bbox"))
        return self.process_crop(crop, det, timestamp=ts, context=context)

    def process_crop(self, crop, detection, timestamp=None, context=None):
        ts = float(timestamp if timestamp is not None else now_seconds())
        det = dict(detection or {})
        if ts - self.last_ocr_ts < self.ocr_interval:
            return {
                "active": True,
                "status": "ocr_throttled",
                "detection": det,
                "instruction": fixed_unknown_result("", "ocr_throttled"),
                "latest_instruction": self.latest_result,
                "instruction_current": False,
            }

        if crop is None or crop.size == 0:
            return {
                "active": True,
                "status": "bad_crop",
                "detection": det,
                "instruction": fixed_unknown_result("", "bad_crop"),
                "latest_instruction": self.latest_result,
                "instruction_current": False,
            }

        # PaddleOCR is loaded lazily so startup does not block the control loop.
        # Keep an initialization failure visible to the HUD, but do not retry it on
        # every video frame (which previously flooded ar_preview.log).
        if self.ocr_reader is None:
            if ts - self.last_ocr_init_attempt_ts < self.ocr_init_retry_interval:
                return {
                    "active": True,
                    "status": "ocr_unavailable",
                    "detection": det,
                    "instruction": fixed_unknown_result("", "ocr_unavailable"),
                    "latest_instruction": self.latest_result,
                    "instruction_current": False,
                    "error": self.ocr_init_error or "PaddleOCR is not ready",
                }
            self.last_ocr_init_attempt_ts = ts
            try:
                self._ensure_ocr()
            except Exception as exc:
                self.ocr_init_error = str(exc)
                return {
                    "active": True,
                    "status": "ocr_unavailable",
                    "detection": det,
                    "instruction": fixed_unknown_result("", "ocr_unavailable"),
                    "latest_instruction": self.latest_result,
                    "instruction_current": False,
                    "error": self.ocr_init_error,
                }
        self.last_ocr_ts = ts
        try:
            ocr = self.ocr_reader.read(crop)
        except Exception as exc:
            self.ocr_init_error = str(exc)
            self.ocr_reader = None
            return {
                "active": True,
                "status": "ocr_unavailable",
                "detection": det,
                "instruction": fixed_unknown_result("", "ocr_unavailable"),
                "latest_instruction": self.latest_result,
                "instruction_current": False,
                "error": self.ocr_init_error,
            }
        text = normalize_text(ocr.get("text"))
        text_conf = float(ocr.get("confidence") or 0.0)
        stable = self._update_stability(text, text_conf, ts)

        response = {
            "active": True,
            "status": "ocr_ready",
            "detection": det,
            "ocr": ocr,
            "stable": stable,
            "instruction": fixed_unknown_result(text, "ocr_ready"),
            "latest_instruction": self.latest_result,
            "instruction_current": False,
        }

        if not text:
            response["status"] = "empty_ocr_text"
            response["instruction"] = fixed_unknown_result("", "empty_ocr_text")
            return response
        if text_conf < self.min_ocr_confidence:
            response["status"] = "low_ocr_confidence"
            response["instruction"] = fixed_unknown_result(text, "low_ocr_confidence")
            response["instruction"]["ocr_confidence"] = text_conf
            return response
        if not stable:
            response["status"] = "waiting_stable_text"
            response["instruction"] = fixed_unknown_result(text, "waiting_stable_text")
            response["instruction"]["ocr_confidence"] = text_conf
            return response

        if self.async_api and self.pending_thread is not None:
            if self.pending_thread.is_alive():
                if self.pending_text == text:
                    response["status"] = "api_pending"
                    response["instruction"] = fixed_unknown_result(text, "api_pending")
                    response["instruction"]["ocr_confidence"] = text_conf
                    response["latest_instruction"] = self.latest_result
                    response["instruction_current"] = False
                    return response
            else:
                finished_text = self.pending_text
                self.pending_thread = None
                self.pending_text = ""
                if finished_text == text:
                    response["status"] = "api_done"
                    response["instruction"] = self.latest_result
                    response["latest_instruction"] = self.latest_result
                    response["instruction_current"] = True
                    return response

        if (
            self.last_api_result_ts > 0.0
            and self.last_api_text == text
            and ts - self.last_api_result_ts <= self.api_result_hold
        ):
            response["status"] = "api_done_recent"
            response["instruction"] = self.latest_result
            response["latest_instruction"] = self.latest_result
            response["instruction_current"] = True
            return response

        cache_key = f"{API_CACHE_VERSION}:{self.interpreter.model}:{text}:{image_ahash(crop)}"
        cached = self._get_cache(cache_key, ts)
        if cached is not None:
            self.latest_result = cached
            response["status"] = "cache_hit"
            response["instruction"] = self.latest_result
            response["latest_instruction"] = self.latest_result
            response["instruction_current"] = True
            return response

        if ts - self.last_api_ts < self.api_cooldown:
            response["status"] = "api_throttled"
            response["instruction"] = fixed_unknown_result(text, "api_throttled")
            response["instruction"]["ocr_confidence"] = text_conf
            return response

        api_context = {
            "detection": {
                "score": det.get("score"),
                "bbox": det.get("bbox"),
                "area_ratio": det.get("area_ratio"),
            },
            **(context or {}),
        }
        if self.async_api:
            self._start_api_thread(text, text_conf, api_context, cache_key, ts)
            response["status"] = "api_pending"
            response["instruction"] = fixed_unknown_result(text, "api_pending")
            response["instruction"]["ocr_confidence"] = text_conf
            return response

        result = self.interpreter.interpret(text, text_conf, api_context)
        finished_ts = now_seconds()
        self.last_api_ts = ts
        self.latest_result = result
        self.last_api_text = text
        self.last_api_result_ts = finished_ts
        self._put_cache(cache_key, result, finished_ts)
        response["status"] = "api_done"
        response["instruction"] = self.latest_result
        response["latest_instruction"] = self.latest_result
        response["instruction_current"] = True
        return response

    def _update_stability(self, text, confidence, ts):
        self.history.append({"text": text, "confidence": float(confidence), "timestamp": float(ts)})
        if not text:
            self.stable_count = 0
            self.last_text = ""
            self.stable_since_ts = 0.0
            return False
        if (
            self.stable_duration_s <= 0.0
            and
            confidence >= self.stable_bypass_confidence
            and len(text) >= self.stable_bypass_min_text_len
        ):
            self.stable_count = max(self.stable_count, self.stable_frames)
            if not self.stable_since_ts:
                self.stable_since_ts = float(ts)
            self.last_text = text
            return True
        if self.last_text and SequenceMatcher(None, self.last_text, text).ratio() >= self.text_similarity:
            self.stable_count += 1
        else:
            self.stable_count = 1
            self.last_text = text
            self.stable_since_ts = float(ts)
        if self.stable_count < self.stable_frames:
            return False
        return float(ts) - float(self.stable_since_ts) >= self.stable_duration_s

    def _get_cache(self, key, ts):
        if self.cache_ttl <= 0.0:
            return None
        item = self.cache.get(key)
        if not item:
            return None
        age = ts - item["timestamp"]
        if age > self.cache_ttl:
            self.cache.pop(key, None)
            return None
        return dict(item["result"])

    def _put_cache(self, key, result, ts):
        if self.cache_ttl <= 0.0:
            return
        self.cache[key] = {"timestamp": float(ts), "result": dict(result)}

    def _start_api_thread(self, text, confidence, context, cache_key, ts):
        if self.pending_thread is not None and self.pending_thread.is_alive():
            return
        self.pending_text = text
        self.last_api_ts = ts

        def worker():
            try:
                result = self.interpreter.interpret(text, confidence, context)
            except Exception as exc:
                result = fixed_unknown_result(text, "api_thread_failed")
                result["api_error"] = str(exc)
            self.latest_result = result
            self._put_cache(cache_key, result, now_seconds())
            self.last_api_text = text
            self.last_api_result_ts = now_seconds()

        self.pending_thread = threading.Thread(target=worker, name="turnsign-aistudio-api", daemon=True)
        self.pending_thread.start()


def _queue_put_latest(output_queue, payload):
    try:
        output_queue.put_nowait(payload)
        return True
    except queue.Full:
        pass
    except Exception:
        return False
    try:
        output_queue.get_nowait()
    except Exception:
        pass
    try:
        output_queue.put_nowait(payload)
        return True
    except Exception:
        return False


def _ocr_worker_main(request_queue, result_queue, stop_event, processor_kwargs, worker_cpu_set):
    limit_ocr_cpu_threads()
    affinity_applied = False

    def apply_worker_scheduling():
        nonlocal affinity_applied
        try:
            target_nice = int(os.environ.get("AR_TURNSIGN_OCR_NICE", "0"))
            os.setpriority(os.PRIO_PROCESS, 0, max(-20, min(19, target_nice)))
        except (AttributeError, OSError, TypeError, ValueError):
            pass
        try:
            from cpu_affinity import apply_current_thread_affinity

            apply_current_thread_affinity(worker_cpu_set, None, "turnsign-ocr-worker")
            affinity_applied = True
        except Exception:
            try:
                cpus = set()
                for chunk in str(worker_cpu_set or "").split(","):
                    chunk = chunk.strip()
                    if not chunk:
                        continue
                    if "-" in chunk:
                        first, last = [int(value.strip()) for value in chunk.split("-", 1)]
                        cpus.update(range(min(first, last), max(first, last) + 1))
                    else:
                        cpus.add(int(chunk))
                if cpus:
                    os.sched_setaffinity(0, cpus)
                    affinity_applied = True
            except (AttributeError, OSError, TypeError, ValueError):
                pass

    def worker_ready_payload():
        try:
            worker_nice = os.getpriority(os.PRIO_PROCESS, 0)
        except (AttributeError, OSError):
            worker_nice = None
        try:
            worker_affinity = sorted(os.sched_getaffinity(0))
        except (AttributeError, OSError):
            worker_affinity = []
        return {
            "type": "worker_ready",
            "timestamp": now_seconds(),
            "nice": worker_nice,
            "affinity": worker_affinity,
            "affinity_applied": affinity_applied,
        }

    kwargs = dict(processor_kwargs or {})
    kwargs["log_func"] = None
    # The process is already the asynchronous boundary.  Running another API
    # thread inside it can leave the response at api_pending forever when the
    # SDK request stalls.  Keep OCR and the bounded API request sequential here.
    kwargs["async_api"] = False
    processor = None
    try:
        processor = TurnSignOcrApiProcessor(**kwargs)
        # Import and initialize PaddleOCR in the worker process, not in the
        # latency-sensitive control process.  This avoids the observed
        # "partially initialized paddle" state and keeps import spikes off the
        # main loop.
        processor._ensure_ocr()
        apply_worker_scheduling()
        _queue_put_latest(result_queue, worker_ready_payload())
    except Exception as exc:
        _queue_put_latest(
            result_queue,
            {
                "type": "worker_error",
                "timestamp": now_seconds(),
                "error": str(exc),
            },
        )

    while not stop_event.is_set():
        try:
            payload = request_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        except Exception:
            continue
        if not isinstance(payload, dict):
            continue
        if payload.get("type") == "stop":
            break
        if processor is None:
            try:
                processor = TurnSignOcrApiProcessor(**kwargs)
                processor._ensure_ocr()
                apply_worker_scheduling()
                _queue_put_latest(result_queue, worker_ready_payload())
            except Exception as exc:
                _queue_put_latest(
                    result_queue,
                    {
                        "type": "result",
                        "timestamp": now_seconds(),
                        "request_id": payload.get("request_id"),
                        "session_id": payload.get("session_id"),
                        "response": {
                            "active": True,
                            "status": "ocr_unavailable",
                            "detection": payload.get("detection") or {},
                            "instruction": fixed_unknown_result("", "not_started"),
                            "error": str(exc),
                        },
                    },
                )
                processor = None
                continue

        try:
            response = processor.process_crop(
                payload.get("crop"),
                payload.get("detection") or {},
                timestamp=payload.get("timestamp"),
                context=payload.get("context") or {},
            )
        except Exception as exc:
            response = {
                "active": True,
                "status": "ocr_unavailable",
                "detection": payload.get("detection") or {},
                "instruction": getattr(processor, "latest_result", fixed_unknown_result("", "not_started")),
                "error": str(exc),
            }
        _queue_put_latest(
            result_queue,
            {
                "type": "result",
                "timestamp": now_seconds(),
                "request_id": payload.get("request_id"),
                "session_id": payload.get("session_id"),
                "response": response,
            },
        )


class AsyncTurnSignOcrApiProcessor:
    """Non-blocking wrapper around TurnSignOcrApiProcessor.

    The control loop only selects/crops the latest TurnSign detection and then
    returns immediately.  PaddleOCR and API interpretation run in a separate
    process pinned to the non-control cores.
    """

    def __init__(
        self,
        worker_cpu_set="0-3",
        mp_start_method=None,
        log_func=None,
        confirm_frames=3,
        confirm_iou=0.30,
        confirm_max_misses=2,
        detection_line_ratio=185.0 / 480.0,
        preconfirm_line_distance_px_480=45.0,
        edge_margin_ratio=0.15,
        confirmed_max_misses=3,
        session_absence_timeout_s=3.0,
        ocr_response_timeout_s=10.0,
        **processor_kwargs,
    ):
        self.log_func = log_func or (lambda _message: None)
        self.worker_cpu_set = str(worker_cpu_set or "").strip()
        self.processor_kwargs = dict(processor_kwargs or {})
        self.processor_kwargs.pop("worker_cpu_set", None)
        self.processor_kwargs.pop("log_func", None)

        self.min_det_score = float(self.processor_kwargs.get("min_det_score", 0.40))
        self.min_area_ratio = float(self.processor_kwargs.get("min_area_ratio", 0.031))
        self.ocr_interval = float(self.processor_kwargs.get("ocr_interval", 0.25))
        self.confirm_frames = max(1, int(confirm_frames))
        self.confirm_iou = clamp(confirm_iou, 0.0, 1.0)
        self.confirm_max_misses = max(0, int(confirm_max_misses))
        self.detection_line_ratio = clamp(detection_line_ratio, 0.05, 0.95)
        self.preconfirm_line_distance_px_480 = max(
            0.0, float(preconfirm_line_distance_px_480))
        self.edge_margin_ratio = clamp(edge_margin_ratio, 0.0, 0.45)
        self.confirmed_max_misses = max(0, int(confirmed_max_misses))
        self.session_absence_timeout_s = max(
            0.0, float(session_absence_timeout_s))
        self.ocr_response_timeout_s = max(
            0.0, float(ocr_response_timeout_s))
        self.confirm_count = 0
        self.confirm_misses = 0
        self.confirm_bbox = None
        self.session_active = False
        self.session_resolved = False
        self.session_id = 0
        self.latest_result = fixed_unknown_result("", "not_started")
        self.latest_response = {
            "active": False,
            "status": "idle",
            "instruction": self.latest_result,
        }
        self.latest_response_ts = 0.0
        self.parent_result_hold = float(
            os.environ.get("AR_TURNSIGN_PARENT_RESULT_HOLD", "3.0")
        )
        self.stable_result_max_age = float(
            os.environ.get("AR_TURNSIGN_STABLE_RESULT_MAX_AGE", "15.0")
        )
        self.sign_absence_reset = float(
            os.environ.get("AR_TURNSIGN_ABSENCE_RESET", "0.75")
        )
        self.last_valid_response = None
        self.last_valid_response_ts = 0.0
        self.last_turnsign_seen_ts = 0.0
        self.session_last_turnsign_seen_ts = 0.0
        self.session_ocr_started_ts = 0.0
        self.last_turnsign_bbox = None
        self.last_position_detection = None
        self.tracking_misses = 0
        self.turnsign_snapshot_crop = None
        self.turnsign_snapshot_detection = None
        self.turnsign_snapshot_ts = 0.0
        self.detection_iou_reset = float(
            os.environ.get("AR_TURNSIGN_DET_IOU_RESET", "0.05")
        )
        self.detection_reset_frames = max(
            1,
            int(os.environ.get("AR_TURNSIGN_DET_RESET_FRAMES", "3")),
        )
        self.bbox_mismatch_candidate = None
        self.bbox_mismatch_count = 0
        self.clear_result_pending = False
        self.min_result_request_id = 0
        self.worker_error = ""
        self.worker_ready = False
        self.worker_nice = None
        self.worker_affinity = []
        self.pending = False
        self.pending_since_ts = 0.0
        # PaddleOCR first inference and the ERNIE/AiStudio API call can both take
        # several seconds on the RK board.  A 4 s timeout was too aggressive and
        # caused the worker to be restarted before the API result could return.
        self.worker_timeout = float(os.environ.get("AR_TURNSIGN_OCR_WORKER_TIMEOUT", "20.0"))
        self.worker_restart_cooldown = float(
            os.environ.get("AR_TURNSIGN_OCR_RESTART_COOLDOWN", "3.0")
        )
        self.last_worker_restart_ts = 0.0
        self.last_submit_ts = 0.0
        self.request_id = 0
        self.pending_request_id = 0

        start_method = mp_start_method or os.environ.get("AR_TURNSIGN_OCR_MP_START", "spawn")
        self.ctx = mp.get_context(start_method)
        # Do not create the process or load PaddleOCR at application startup.
        # The third valid TurnSign observation starts this worker as a prewarm
        # step; the image is submitted only after the box top crosses the line
        # while its horizontal center is outside neither edge zone.
        self.request_queue = None
        self.result_queue = None
        self.stop_event = None
        self.process_handle = None

    def _start_worker(self):
        if self._worker_alive():
            return True
        self.request_queue = self.ctx.Queue(maxsize=1)
        self.result_queue = self.ctx.Queue(maxsize=3)
        self.stop_event = self.ctx.Event()
        self.process_handle = self.ctx.Process(
            target=_ocr_worker_main,
            name="turnsign-ocr-worker",
            args=(
                self.request_queue,
                self.result_queue,
                self.stop_event,
                self.processor_kwargs,
                self.worker_cpu_set,
            ),
            daemon=True,
        )
        try:
            self.process_handle.start()
        except Exception as exc:
            self.process_handle = None
            self.worker_ready = False
            self.worker_error = f"worker start failed: {exc}"
            self.log_func(f"turnsign OCR {self.worker_error}")
            return False
        self.worker_error = ""
        self.log_func(f"turnsign OCR worker started pid={self.process_handle.pid} cpuset={self.worker_cpu_set or '-'}")
        return True

    def _stop_worker(self, terminate=False, graceful_timeout=0.8):
        try:
            if getattr(self, "stop_event", None) is not None:
                self.stop_event.set()
            if getattr(self, "request_queue", None) is not None:
                _queue_put_latest(self.request_queue, {"type": "stop"})
        except Exception:
            pass
        try:
            if self.process_handle is not None:
                self.process_handle.join(timeout=max(0.0, float(graceful_timeout)))
                if self.process_handle.is_alive() and terminate:
                    self.process_handle.terminate()
                    self.process_handle.join(timeout=0.5)
        except Exception:
            pass

    def _restart_worker(self, reason):
        self.log_func(f"turnsign OCR worker restart: {reason}")
        self._stop_worker(terminate=True, graceful_timeout=0.3)
        self.pending = False
        self.pending_since_ts = 0.0
        self.pending_request_id = 0
        self.worker_ready = False
        self.worker_error = str(reason)
        self.last_worker_restart_ts = now_seconds()
        self._start_worker()

    def _worker_alive(self):
        process = getattr(self, "process_handle", None)
        return process is not None and process.is_alive()

    def _worker_exit_error(self):
        process = getattr(self, "process_handle", None)
        if process is None:
            return self.worker_error or "OCR worker was not started"
        return f"OCR worker exited unexpectedly (exitcode={process.exitcode})"

    @staticmethod
    def is_turnsign_detection(det):
        return TurnSignOcrApiProcessor.is_turnsign_detection(det)

    @staticmethod
    def _frame_size(frame):
        if frame is not None and hasattr(frame, "shape") and len(frame.shape) >= 2:
            return int(frame.shape[0]), int(frame.shape[1])
        return 480, 640

    def _candidate_gate(self, det, frame=None):
        height, _width = self._frame_size(frame)
        area = self._detection_area_ratio(det, frame)
        if area < self.min_area_ratio:
            return False, "turnsign_too_small"
        try:
            top = float((det or {}).get("bbox")[1])
        except (TypeError, ValueError, IndexError):
            return False, "turnsign_bad_bbox"
        line_y = float(height) * self.detection_line_ratio
        max_gap = (
            self.preconfirm_line_distance_px_480 *
            float(max(1, height)) / 480.0)
        # Image y grows downward. At 480p the line is y=185, so only a box
        # whose top has gone below y=230 is too far past the detection line.
        if top > line_y + max_gap:
            return False, "turnsign_too_far"
        return True, "turnsign_candidate"

    def _select_turnsign_with_status(self, detections, frame=None):
        """Select one valid TurnSign and retain why rejected boxes failed."""
        candidates = []
        rejected = []
        for det in detections or []:
            if not self.is_turnsign_detection(det):
                continue
            try:
                score = float(det.get("score", 0.0))
                area = float(det.get("area_ratio", 0.0))
            except Exception:
                continue
            if score < self.min_det_score:
                rejected.append("turnsign_low_score")
                continue
            valid, status = self._candidate_gate(det, frame)
            if not valid:
                rejected.append(status)
                continue
            candidates.append((score + area * 4.0, det))
        if not candidates:
            priority = (
                "turnsign_too_small", "turnsign_too_far",
                "turnsign_low_score", "turnsign_bad_bbox")
            status = next(
                (item for item in priority if item in rejected),
                "no_turnsign")
            return None, status
        candidates.sort(key=lambda item: item[0], reverse=True)
        if self.last_turnsign_bbox is not None:
            tracked = max(
                candidates,
                key=lambda item: self._bbox_iou(
                    self.last_turnsign_bbox,
                    item[1].get("bbox"),
                ),
            )
            if self._bbox_iou(self.last_turnsign_bbox, tracked[1].get("bbox")) >= self.detection_iou_reset:
                return tracked[1], "turnsign_candidate"
        return candidates[0][1], "turnsign_candidate"

    def select_turnsign(self, detections, frame=None):
        selected, _status = self._select_turnsign_with_status(
            detections, frame)
        return selected

    def _select_tracking_continuation(self, detections, frame=None):
        """After confirmation, tolerate the same sign changing class."""
        if self.last_turnsign_bbox is None:
            return None
        candidates = []
        for det in detections or []:
            label = str(
                det.get("label") or det.get("category") or ""
            ).strip().lower().replace("_", "").replace("-", "")
            if label not in {"door", "endsign", "beginsign"}:
                continue
            try:
                score = float(det.get("score", 0.0))
            except (TypeError, ValueError):
                continue
            if score < self.min_det_score:
                continue
            valid, _status = self._candidate_gate(det, frame)
            if not valid:
                continue
            overlap = self._bbox_iou(
                self.last_turnsign_bbox, det.get("bbox"))
            if overlap < self.detection_iou_reset:
                continue
            candidates.append((overlap, score, det))
        if not candidates:
            return None
        return max(candidates, key=lambda item: (item[0], item[1]))[2]

    def _detection_area_ratio(self, det, frame=None):
        try:
            area = float((det or {}).get("area_ratio", 0.0))
        except (TypeError, ValueError):
            area = 0.0
        if area > 0.0:
            return area
        try:
            left, top, right, bottom = [float(value) for value in det.get("bbox")]
            frame_h, frame_w = frame.shape[:2]
        except (AttributeError, TypeError, ValueError):
            return 0.0
        box_area = max(0.0, right - left) * max(0.0, bottom - top)
        return box_area / float(max(1, int(frame_h) * int(frame_w)))

    def _snapshot_size_reached(self, det, frame=None):
        return self._detection_area_ratio(det, frame) >= self.min_area_ratio

    def _reset_confirmation(self):
        self.confirm_count = 0
        self.confirm_misses = 0
        self.confirm_bbox = None

    def _observe_for_confirmation(self, det):
        """Count any three consecutive valid TurnSign observations."""
        bbox = (det or {}).get("bbox")
        self.confirm_count = min(
            self.confirm_frames, self.confirm_count + 1)
        self.confirm_misses = 0
        self.confirm_bbox = list(bbox) if bbox is not None else None

        if self.session_active or self.confirm_count < self.confirm_frames:
            return False
        self.session_active = True
        self.session_resolved = False
        self.session_ocr_started_ts = 0.0
        self.session_id += 1
        self._start_worker()
        return True

    def _observe_confirmation_miss(self):
        if self.session_active or self.confirm_count <= 0:
            return
        # Confirmation means three genuinely consecutive frames. Any miss or
        # rejected too-small/too-far box restarts the count immediately.
        self._reset_confirmation()

    def _position_info(self, det, frame=None, detection_fresh=True):
        height, width = self._frame_size(frame)
        line_y = float(height) * self.detection_line_ratio
        left_edge = float(width) * self.edge_margin_ratio
        right_edge = float(width) * (1.0 - self.edge_margin_ratio)
        info = {
            "detection_line_y": line_y,
            "left_edge_x": left_edge,
            "right_edge_x": right_edge,
            "detection_fresh": bool(detection_fresh),
            "tracking_misses": int(self.tracking_misses),
        }
        if det is None:
            info["control_phase"] = (
                "turnsign_missing_hold"
                if self.tracking_misses <= self.confirmed_max_misses
                else "turnsign_missing_stop")
            return info
        try:
            left, top, right, bottom = [
                float(value) for value in det.get("bbox")[:4]]
        except (TypeError, ValueError, IndexError):
            info["control_phase"] = "turnsign_missing_stop"
            return info
        center_x = 0.5 * (left + right)
        info.update({
            "bbox_top": top,
            "bbox_center_x": center_x,
            "bbox_bottom": bottom,
        })
        if top >= line_y:
            if center_x < left_edge:
                info["edge_side"] = "left"
                info["control_phase"] = "turnsign_edge_over_line"
            elif center_x > right_edge:
                info["edge_side"] = "right"
                info["control_phase"] = "turnsign_edge_over_line"
            else:
                info["control_phase"] = "turnsign_position_ready"
            return info
        if center_x < left_edge:
            info["edge_side"] = "left"
            info["control_phase"] = "turnsign_edge_left"
        elif center_x > right_edge:
            info["edge_side"] = "right"
            info["control_phase"] = "turnsign_edge_right"
        else:
            info["control_phase"] = "turnsign_approach"
        return info

    @staticmethod
    def _response_direction(response):
        response = response if isinstance(response, dict) else {}
        instruction = response.get("instruction") or {}
        return str(
            instruction.get("direction")
            or instruction.get("preferred_branch")
            or ""
        )

    @staticmethod
    def _bbox_iou(first, second):
        try:
            ax0, ay0, ax1, ay1 = [float(value) for value in first]
            bx0, by0, bx1, by1 = [float(value) for value in second]
        except (TypeError, ValueError):
            return 0.0
        inter_w = max(0.0, min(ax1, bx1) - max(ax0, bx0))
        inter_h = max(0.0, min(ay1, by1) - max(ay0, by0))
        intersection = inter_w * inter_h
        area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
        area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
        union = area_a + area_b - intersection
        return intersection / union if union > 0.0 else 0.0

    def _stable_response(self, det, ts, refresh_pending=False, refresh_status=None):
        response = self.last_valid_response
        if not isinstance(response, dict):
            return None
        age = ts - self.last_valid_response_ts
        if self.stable_result_max_age > 0.0 and age > self.stable_result_max_age:
            self.last_valid_response = None
            self.last_valid_response_ts = 0.0
            self.latest_result = fixed_unknown_result("", "stable_result_expired")
            self.clear_result_pending = True
            return None
        if not response.get("instruction_current"):
            return None
        if self._response_direction(response) not in {"left", "right"}:
            return None

        held = dict(response)
        held["active"] = True
        held["status"] = "api_done_held"
        held["detection"] = det
        held["worker_ready"] = self.worker_ready
        held["error"] = self.worker_error or None
        held["refresh_pending"] = bool(refresh_pending)
        held["instruction_current"] = False
        held["display_result_valid"] = True
        held["turnsign_resolved"] = bool(self.session_resolved)
        held["control_phase"] = (
            "turnsign_consumed" if self.session_resolved
            else "turnsign_ocr_wait")
        held["session_id"] = self.session_id
        held["result_age"] = max(0.0, float(age))
        if refresh_status:
            held["refresh_status"] = str(refresh_status)
        else:
            held.pop("refresh_status", None)
        return held

    def _invalidate_stable_result(self):
        self.last_valid_response = None
        self.last_valid_response_ts = 0.0
        self.latest_result = fixed_unknown_result("", "turnsign_changed")
        self.latest_response = {
            "active": False,
            "status": "turnsign_changed",
            "instruction": self.latest_result,
            "latest_instruction": self.latest_result,
            "instruction_current": False,
        }
        self.latest_response_ts = 0.0
        self.min_result_request_id = max(self.min_result_request_id, self.request_id + 1)
        self.pending = False
        self.pending_since_ts = 0.0
        self.pending_request_id = 0
        self.session_active = False
        self.session_resolved = False
        self._reset_confirmation()
        self.bbox_mismatch_candidate = None
        self.bbox_mismatch_count = 0
        self.last_position_detection = None
        self.tracking_misses = 0
        self.turnsign_snapshot_crop = None
        self.turnsign_snapshot_detection = None
        self.turnsign_snapshot_ts = 0.0
        self.session_last_turnsign_seen_ts = 0.0
        self.session_ocr_started_ts = 0.0

    def _abort_unresolved_session(self, status, control_phase, timestamp):
        """Release one unresolved sign session and reject its late result."""
        self.min_result_request_id = max(
            self.min_result_request_id, self.request_id + 1)
        self.pending = False
        self.pending_since_ts = 0.0
        self.pending_request_id = 0
        self.session_active = False
        self.session_resolved = False
        self._reset_confirmation()
        self.bbox_mismatch_candidate = None
        self.bbox_mismatch_count = 0
        self.last_position_detection = None
        self.tracking_misses = 0
        self.turnsign_snapshot_crop = None
        self.turnsign_snapshot_detection = None
        self.turnsign_snapshot_ts = 0.0
        self.session_last_turnsign_seen_ts = 0.0
        self.session_ocr_started_ts = 0.0
        self.latest_result = fixed_unknown_result("", status)
        response = {
            "active": False,
            "status": str(status),
            "control_phase": str(control_phase),
            "instruction": self.latest_result,
            "latest_instruction": self.latest_result,
            "instruction_current": False,
            "session_id": self.session_id,
            "session_active": False,
            "turnsign_resolved": False,
            "clear_result": True,
        }
        self.latest_response = dict(response)
        self.latest_response_ts = float(timestamp)
        self.clear_result_pending = False
        self.log_func(
            f"turnsign OCR session {self.session_id} exited: {status}")
        return response

    def _capture_turnsign_snapshot(self, frame, det, timestamp):
        """Freeze the first correctly positioned crop and reuse it."""
        if self.turnsign_snapshot_crop is None:
            crop = crop_bbox(frame, (det or {}).get("bbox"))
            if crop is None or crop.size == 0:
                return None
            self.turnsign_snapshot_crop = np.ascontiguousarray(crop).copy()
            self.turnsign_snapshot_detection = dict(det or {})
            self.turnsign_snapshot_ts = float(timestamp)
        return self.turnsign_snapshot_crop

    def _refresh_turnsign_snapshot(self, frame, det, timestamp):
        """Replace a failed OCR crop once a fresh, correctly placed box exists."""
        self.turnsign_snapshot_crop = None
        self.turnsign_snapshot_detection = None
        self.turnsign_snapshot_ts = 0.0
        return self._capture_turnsign_snapshot(frame, det, timestamp)

    def close(self):
        self._stop_worker(terminate=True, graceful_timeout=10.0)

    def _drain_results(self):
        result_received = False
        while True:
            try:
                item = self.result_queue.get_nowait()
            except queue.Empty:
                break
            except Exception:
                break
            if not isinstance(item, dict):
                continue
            item_type = item.get("type")
            if item_type == "worker_ready":
                self.worker_ready = True
                self.worker_error = ""
                self.worker_nice = item.get("nice")
                self.worker_affinity = list(item.get("affinity") or [])
                self.log_func(
                    "turnsign OCR worker ready "
                    f"nice={self.worker_nice} affinity={self.worker_affinity or '-'}"
                )
            elif item_type == "worker_error":
                self.worker_ready = False
                self.worker_error = str(item.get("error") or "")
                self.latest_response = {
                    "active": True,
                    "status": "ocr_unavailable",
                    "instruction": self.latest_result,
                    "latest_instruction": self.latest_result,
                    "instruction_current": False,
                    "error": self.worker_error,
                }
                self.log_func(f"turnsign OCR worker init error: {self.worker_error}")
            elif item_type == "result":
                try:
                    item_request_id = int(item.get("request_id") or 0)
                except (TypeError, ValueError):
                    item_request_id = 0
                try:
                    item_session_id = int(item.get("session_id") or 0)
                except (TypeError, ValueError):
                    item_session_id = 0
                if (
                    not item_request_id
                    or item_request_id != self.pending_request_id
                    or item_session_id != self.session_id
                ):
                    continue
                response = dict(item.get("response") or {})
                instruction = response.get("instruction")
                if isinstance(instruction, dict) and bool(response.get("instruction_current", False)):
                    self.latest_result = dict(instruction)
                    if self._response_direction(response) in {"left", "right"}:
                        self.last_valid_response = dict(response)
                        self.last_valid_response_ts = now_seconds()
                        self.session_resolved = True
                else:
                    response.setdefault("latest_instruction", self.latest_result)
                self.latest_response = response
                self.latest_response_ts = now_seconds()
                self.pending = False
                self.pending_since_ts = 0.0
                self.pending_request_id = 0
                result_received = True
        return result_received

    def process(self, frame, detections, timestamp=None, context=None):
        ts = float(timestamp if timestamp is not None else now_seconds())
        clear_result = bool(self.clear_result_pending)
        self.clear_result_pending = False
        result_received = self._drain_results()
        det, candidate_status = self._select_turnsign_with_status(
            detections, frame)
        fresh_turnsign_detected = det is not None
        continuation = False
        if det is None and self.session_active and not self.session_resolved:
            det = self._select_tracking_continuation(detections, frame)
            continuation = det is not None
            if continuation:
                candidate_status = "turnsign_class_continuation"

        if det is None:
            if self.session_active and not self.session_resolved:
                self.tracking_misses += 1
            else:
                self._observe_confirmation_miss()
            if (
                self.last_turnsign_seen_ts > 0.0
                and ts - self.last_turnsign_seen_ts >= self.sign_absence_reset
                and not result_received
                and (not self.session_active or self.session_resolved)
            ):
                self._invalidate_stable_result()
                self.last_turnsign_seen_ts = 0.0
                self.last_turnsign_bbox = None
                self.last_position_detection = None
                self.tracking_misses = 0
                clear_result = True
        else:
            current_bbox = det.get("bbox")
            self.tracking_misses = 0
            self.last_position_detection = dict(det)
            self.last_turnsign_bbox = (
                list(current_bbox) if current_bbox is not None else None)
            self.last_turnsign_seen_ts = ts
            if fresh_turnsign_detected:
                self.session_last_turnsign_seen_ts = ts
            if not self.session_active:
                self._observe_for_confirmation(det)

        if self.session_active and not self.session_resolved:
            if (
                self.session_absence_timeout_s > 0.0
                and self.session_last_turnsign_seen_ts > 0.0
                and ts - self.session_last_turnsign_seen_ts >=
                self.session_absence_timeout_s
            ):
                return self._abort_unresolved_session(
                    "turnsign_exit_no_sign_3s",
                    "turnsign_exit_no_sign",
                    ts,
                )
            if (
                self.ocr_response_timeout_s > 0.0
                and self.session_ocr_started_ts > 0.0
                and ts - self.session_ocr_started_ts >=
                self.ocr_response_timeout_s
            ):
                return self._abort_unresolved_session(
                    "turnsign_exit_ocr_timeout_10s",
                    "turnsign_exit_ocr_timeout",
                    ts,
                )

        detection_fresh = det is not None
        position_det = det
        if (
            self.session_active
            and not self.session_resolved
            and position_det is None
            and self.tracking_misses <= self.confirmed_max_misses
        ):
            position_det = self.last_position_detection

        if self.session_resolved:
            position_info = {"control_phase": "turnsign_consumed"}
        elif not self.session_active:
            position_info = {"control_phase": (
                "turnsign_confirming" if det is not None
                else candidate_status)}
        elif self.turnsign_snapshot_crop is not None:
            position_info = {"control_phase": "turnsign_ocr_wait"}
        else:
            position_info = self._position_info(
                position_det, frame, detection_fresh=detection_fresh)

        position_ready = (
            position_info.get("control_phase") ==
            "turnsign_position_ready")
        if (
            self.session_active
            and not self.session_resolved
            and position_ready
            and self.turnsign_snapshot_crop is None
        ):
            self._capture_turnsign_snapshot(frame, position_det, ts)
            position_info["control_phase"] = "turnsign_ocr_wait"

        # Keep the old field for callers that already consume it, while the
        # new field names the actual line-and-edge positioning condition.
        stop_size_reached = bool(position_ready)
        work_det = (
            position_det or self.turnsign_snapshot_detection or
            self.last_position_detection)

        base = {
            "active": bool(det is not None or self.session_active),
            "detection": work_det,
            "instruction": fixed_unknown_result("", "ocr_pending"),
            "latest_instruction": self.latest_result,
            "instruction_current": False,
            "worker_ready": self.worker_ready,
            "error": self.worker_error or None,
            "clear_result": clear_result,
            "confirm_count": self.confirm_count,
            "confirm_frames": self.confirm_frames,
            "session_id": self.session_id if self.session_active else None,
            "session_active": self.session_active,
            "turnsign_resolved": self.session_resolved,
            "candidate_status": candidate_status,
            "class_continuation": bool(continuation),
            "stop_size_reached": stop_size_reached,
            "position_ready": bool(position_ready),
            "area_ratio": (
                self._detection_area_ratio(det, frame) if det is not None else None
            ),
            "snapshot_captured": self.turnsign_snapshot_crop is not None,
            "snapshot_timestamp": self.turnsign_snapshot_ts or None,
            **position_info,
        }

        if result_received:
            response = dict(self.latest_response or {})
            response.setdefault("active", True)
            response.setdefault("detection", work_det)
            response.setdefault("instruction", fixed_unknown_result("", "worker_result"))
            response.setdefault("latest_instruction", self.latest_result)
            response.setdefault("instruction_current", False)
            response["worker_ready"] = self.worker_ready
            response["clear_result"] = clear_result
            response["session_id"] = self.session_id
            response["session_active"] = self.session_active
            response["turnsign_resolved"] = self.session_resolved
            response["control_phase"] = (
                "turnsign_consumed" if self.session_resolved
                else base.get("control_phase"))
            for key in (
                "detection_line_y", "left_edge_x", "right_edge_x",
                "bbox_top", "bbox_center_x", "edge_side",
                "detection_fresh", "tracking_misses",
            ):
                if key in base:
                    response[key] = base[key]
            response["snapshot_captured"] = self.turnsign_snapshot_crop is not None
            response["snapshot_timestamp"] = self.turnsign_snapshot_ts or None
            if self.worker_error:
                response["error"] = self.worker_error
            if (
                response.get("instruction_current")
                and self._response_direction(response) in {"left", "right"}
            ):
                return response

            # Keep the car stopped and retry OCR.  For a terminal failed read,
            # use a new frame instead of retrying the same frozen bad crop
            # forever.  Do not refresh while text/API stability is still in
            # progress because those stages intentionally reuse one image.
            retry_status = str(response.get("status") or "")
            terminal_retry = retry_status in {
                "bad_crop", "empty_ocr_text", "low_ocr_confidence",
                "ocr_unavailable", "api_done", "api_done_recent",
                "cache_hit",
            }
            fresh_position = (
                det is not None
                and self._position_info(det, frame).get("control_phase") ==
                "turnsign_position_ready"
            )
            if terminal_retry and fresh_position:
                refreshed = self._refresh_turnsign_snapshot(frame, det, ts)
                response["snapshot_refreshed"] = refreshed is not None
                response["snapshot_captured"] = refreshed is not None
                response["snapshot_timestamp"] = (
                    self.turnsign_snapshot_ts or None)
            held = self._stable_response(
                work_det,
                ts,
                refresh_pending=False,
                refresh_status=response.get("status"),
            )
            if held is not None:
                return held
            return response

        if (
            self.parent_result_hold > 0.0
            and ts - self.latest_response_ts <= self.parent_result_hold
        ):
            held = self._stable_response(work_det, ts)
            if held is not None:
                return held

        if det is None and not self.session_active:
            response = dict(base)
            response["status"] = (
                "turnsign_confirmation_paused"
                if self.confirm_count > 0 else candidate_status
            )
            return response

        if not self.session_active:
            response = dict(base)
            response["status"] = "turnsign_confirming"
            return response

        if self.session_resolved:
            response = dict(base)
            response["active"] = False
            response["status"] = "route_ready_held"
            response["control_phase"] = "turnsign_consumed"
            response["instruction"] = self.latest_result
            response["latest_instruction"] = self.latest_result
            return response

        if not self._worker_alive():
            self.pending = False
            self.pending_since_ts = 0.0
            self.pending_request_id = 0
            error = self._worker_exit_error()
            self.worker_ready = False
            self.worker_error = error
            if ts - self.last_worker_restart_ts >= self.worker_restart_cooldown:
                self._restart_worker(error)
            response = dict(base)
            response["status"] = "ocr_worker_restarting"
            response["worker_ready"] = False
            response["error"] = error
            return response

        if self.turnsign_snapshot_crop is None:
            response = dict(base)
            response["status"] = (
                "waiting_stop_size" if self.worker_ready else "ocr_prewarming"
            )
            return response

        if not self.worker_ready and not self.worker_error:
            response = dict(base)
            response["status"] = "snapshot_waiting_worker"
            return response

        if self.pending:
            if self.worker_timeout > 0.0 and ts - self.pending_since_ts > self.worker_timeout:
                reason = f"pending>{self.worker_timeout:.1f}s"
                self._restart_worker(reason)
                response = dict(base)
                response["status"] = "ocr_worker_restarted"
                response["worker_ready"] = False
                response["error"] = reason
                return response
            held = self._stable_response(
                work_det,
                ts,
                refresh_pending=True,
                refresh_status="ocr_pending",
            )
            if held is not None:
                return held
            response = dict(base)
            response["status"] = "ocr_pending"
            return response
        if ts - self.last_submit_ts < self.ocr_interval:
            held = self._stable_response(
                work_det,
                ts,
                refresh_pending=False,
                refresh_status="ocr_throttled",
            )
            if held is not None:
                return held
            response = dict(base)
            response["active"] = True
            response["detection"] = work_det
            response["instruction"] = fixed_unknown_result("", "ocr_throttled")
            response["latest_instruction"] = self.latest_result
            response["instruction_current"] = False
            response["status"] = "ocr_throttled"
            response["worker_ready"] = self.worker_ready
            if self.worker_error:
                response["error"] = self.worker_error
            return response

        frozen_crop = self.turnsign_snapshot_crop
        if frozen_crop is None or frozen_crop.size == 0:
            response = dict(base)
            response["status"] = "bad_crop"
            return response
        base["snapshot_captured"] = True
        base["snapshot_timestamp"] = self.turnsign_snapshot_ts

        self.request_id += 1
        payload = {
            "type": "ocr",
            "request_id": self.request_id,
            "session_id": self.session_id,
            "timestamp": ts,
            "crop": frozen_crop,
            "detection": dict(self.turnsign_snapshot_detection or work_det or {}),
            "context": context or {},
        }
        queued = _queue_put_latest(self.request_queue, payload)
        if not queued:
            response = dict(base)
            response["status"] = "ocr_queue_error"
            response["error"] = "failed to enqueue OCR request"
            held = self._stable_response(
                work_det,
                ts,
                refresh_pending=False,
                refresh_status="ocr_queue_error",
            )
            if held is not None:
                held["error"] = response["error"]
                return held
            return response
        self.pending = True
        self.pending_since_ts = ts
        self.pending_request_id = self.request_id
        self.last_submit_ts = ts
        if self.session_ocr_started_ts <= 0.0:
            self.session_ocr_started_ts = ts
        held = self._stable_response(
            work_det,
            ts,
            refresh_pending=True,
            refresh_status="ocr_submitted",
        )
        if held is not None:
            return held
        response = dict(base)
        response["status"] = "ocr_submitted" if self.worker_ready else "ocr_retry_submitted"
        return response

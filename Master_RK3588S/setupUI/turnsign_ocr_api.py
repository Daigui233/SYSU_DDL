import argparse
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


DEFAULT_API_TYPE = "aistudio"
DEFAULT_MODEL = "ernie-3.5"
API_CACHE_VERSION = os.environ.get("AR_TURNSIGN_API_CACHE_VERSION", "api_v2")

DEFAULT_SYSTEM_PROMPT = """你是智能车路牌文字解析器。你的任务是把 OCR 识别到的中文路牌文字交给大模型理解，并转换成固定 JSON，供智能车路线规划模块使用。

必须遵守：
1. 只根据 OCR 原文判断，不要编造 OCR 原文里没有出现的方向、障碍、事故或通行状态。
2. 如果文字不完整、语义不确定、与路线无关，输出 valid=false 且 action="unknown"。
3. 不要套用额外的本地规则或示例映射；直接基于 OCR 原文语义填写字段。
4. 只能输出一个 JSON 对象，不要 Markdown，不要解释文字。

输出字段固定如下：
{
  "valid": false,
  "instruction_type": "unknown",
  "action": "unknown",
  "preferred_branch": "unknown",
  "avoid_branches": [],
  "branch_status": {
    "left": "unknown",
    "right": "unknown",
    "front": "unknown"
  },
  "execute_timing": "unknown",
  "confidence": 0.0,
  "reason": "简短原因",
  "source_text": "OCR原文"
}

字段取值约束：
- valid: true 或 false
- instruction_type: "branch_choice" | "traffic_warning" | "speed_control" | "stop" | "unknown"
- action: "choose_branch" | "avoid_branch" | "slow_down" | "stop" | "keep_lane" | "unknown"
- preferred_branch: "left" | "right" | "front" | "straight" | "unknown"
- avoid_branches: 数组，只能包含 "left"、"right"、"front"、"straight"
- branch_status.left/right/front: "passable" | "blocked" | "accident" | "obstacle" | "closed" | "unknown"
- execute_timing: "now" | "next_fork" | "after_sign" | "unknown"
- confidence: 0 到 1 之间的小数

字段含义由大模型根据 OCR 原文语义自行判断；如果无法判断，填 unknown 或空数组。
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


def fixed_unknown_result(source_text="", reason="unknown"):
    return {
        "valid": False,
        "instruction_type": "unknown",
        "action": "unknown",
        "preferred_branch": "unknown",
        "avoid_branches": [],
        "branch_status": {
            "left": "unknown",
            "right": "unknown",
            "front": "unknown",
        },
        "execute_timing": "unknown",
        "confidence": 0.0,
        "reason": str(reason),
        "source_text": str(source_text or ""),
    }


def normalize_instruction(payload, source_text=""):
    if not isinstance(payload, dict):
        return fixed_unknown_result(source_text, "api_invalid_json")

    result = fixed_unknown_result(source_text, "normalized")
    result.update(payload)

    instruction_types = {"branch_choice", "traffic_warning", "speed_control", "stop", "unknown"}
    actions = {"choose_branch", "avoid_branch", "slow_down", "stop", "keep_lane", "unknown"}
    branches = {"left", "right", "front", "straight", "unknown"}
    status_values = {"passable", "blocked", "accident", "obstacle", "closed", "unknown"}
    timings = {"now", "next_fork", "after_sign", "unknown"}

    result["valid"] = bool(result.get("valid", False))

    result["instruction_type"] = str(result.get("instruction_type", "unknown")).strip()
    if result["instruction_type"] not in instruction_types:
        result["instruction_type"] = "unknown"

    result["action"] = str(result.get("action", "unknown")).strip()
    if result["action"] not in actions:
        result["action"] = "unknown"

    result["preferred_branch"] = str(result.get("preferred_branch", "unknown")).strip()
    if result["preferred_branch"] not in branches:
        result["preferred_branch"] = "unknown"

    avoid = result.get("avoid_branches") or []
    if not isinstance(avoid, list):
        avoid = []
    result["avoid_branches"] = [
        str(item).strip() for item in avoid if str(item).strip() in branches - {"unknown"}
    ]

    branch_status = result.get("branch_status") or {}
    if not isinstance(branch_status, dict):
        branch_status = {}
    result["branch_status"] = {
        "left": str(branch_status.get("left", "unknown")).strip(),
        "right": str(branch_status.get("right", "unknown")).strip(),
        "front": str(branch_status.get("front", "unknown")).strip(),
    }
    for key, value in list(result["branch_status"].items()):
        if value not in status_values:
            result["branch_status"][key] = "unknown"

    result["execute_timing"] = str(result.get("execute_timing", "unknown")).strip()
    if result["execute_timing"] not in timings:
        result["execute_timing"] = "unknown"

    try:
        result["confidence"] = clamp(float(result.get("confidence", 0.0)), 0.0, 1.0)
    except Exception:
        result["confidence"] = 0.0

    result["reason"] = str(result.get("reason", ""))[:120]
    result["source_text"] = str(result.get("source_text") or source_text or "")[:300]

    if result["action"] == "unknown" or result["instruction_type"] == "unknown":
        result["valid"] = False
    if result["confidence"] < 0.2:
        result["valid"] = False
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
        timeout=2.0,
        temperature=0.1,
        system_prompt=None,
        log_func=None,
        api_key=None,
        api_url=None,
    ):
        # api_key/api_url are kept only for old caller compatibility; the active backend is ERNIE SDK + AI Studio.
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
        self.timeout = float(timeout)
        # AI Studio currently rejects 0.0; keep the output deterministic while
        # remaining inside its accepted interval (0, 1].
        self.temperature = max(0.01, min(1.0, float(temperature)))
        self.system_prompt = system_prompt or os.environ.get("AR_TURNSIGN_PROMPT") or DEFAULT_SYSTEM_PROMPT
        self.api_type = DEFAULT_API_TYPE
        self.log_func = log_func or (lambda message: None)
        if api_url:
            self.log_func("turnsign api_url ignored: ERNIE SDK uses api_type='aistudio'")

    def enabled(self):
        return bool(self.access_token)

    def interpret(self, ocr_text, ocr_confidence=None, context=None):
        source_text = normalize_text(ocr_text)
        if not source_text:
            return fixed_unknown_result("", "empty_ocr_text")
        if not self.enabled():
            result = fixed_unknown_result(source_text, "missing_access_token")
            result["ocr_confidence"] = float(ocr_confidence or 0.0)
            result["api_type"] = self.api_type
            result["api_model"] = self.model
            result["api_error"] = "missing_access_token"
            return result

        user_payload = self._build_user_payload(source_text, ocr_confidence, context)
        try:
            content = self._call_erniebot(user_payload)
        except ImportError as exc:
            result = fixed_unknown_result(source_text, "erniebot_import_failed")
            result["api_error"] = str(exc)
            result["install_hint"] = "pip install erniebot"
            result["api_type"] = self.api_type
            result["api_model"] = self.model
            result["ocr_confidence"] = float(ocr_confidence or 0.0)
            return result
        except Exception as exc:
            self.log_func(f"turnsign ERNIE SDK call failed: {exc}")
            result = fixed_unknown_result(source_text, "api_request_failed")
            result["api_error"] = str(exc)
            result["api_type"] = self.api_type
            result["api_model"] = self.model
            result["ocr_confidence"] = float(ocr_confidence or 0.0)
            return result

        payload = safe_json_loads(content)
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

    def _call_erniebot(self, user_payload):
        try:
            import erniebot
        except Exception as exc:
            raise ImportError(f"erniebot import failed: {exc}") from exc

        erniebot.api_type = self.api_type
        erniebot.access_token = self.access_token

        content = (
            f"{self.system_prompt}\n\n"
            "输入数据如下：\n"
            f"{user_payload}\n\n"
            "再次强调：只能输出一个 JSON 对象。"
        )
        kwargs = {
            "_config_": {
                "api_type": self.api_type,
                "access_token": self.access_token,
            },
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
        }
        if self.temperature is not None:
            kwargs["temperature"] = self.temperature

        try:
            response = erniebot.ChatCompletion.create(**kwargs)
        except TypeError:
            kwargs.pop("temperature", None)
            response = erniebot.ChatCompletion.create(**kwargs)
        return self._extract_content(response)

    @staticmethod
    def _build_user_payload(ocr_text, ocr_confidence=None, context=None):
        payload = {
            "ocr_text": ocr_text,
            "ocr_confidence": float(ocr_confidence or 0.0),
            "context": context or {},
            "required_output": {
                "format": "json_object_only",
                "fields": [
                    "valid",
                    "instruction_type",
                    "action",
                    "preferred_branch",
                    "avoid_branches",
                    "branch_status",
                    "execute_timing",
                    "confidence",
                    "reason",
                    "source_text",
                ],
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
        min_det_score=0.35,
        min_area_ratio=0.001,
        min_ocr_confidence=0.45,
        stable_frames=2,
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
        self.stable_count = 0
        self.history = deque(maxlen=8)
        self.cache = {}
        self.pending_thread = None
        self.pending_text = ""
        self.last_api_text = ""
        self.last_api_result_ts = 0.0
        self.api_result_hold = float(os.environ.get("AR_TURNSIGN_API_RESULT_HOLD", "5.0"))
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
            self.async_api
            and self.last_api_result_ts > 0.0
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
        self.last_api_ts = ts
        self.latest_result = result
        self.last_api_text = text
        self.last_api_result_ts = ts
        self._put_cache(cache_key, result, ts)
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
            return False
        if self.last_text and SequenceMatcher(None, self.last_text, text).ratio() >= self.text_similarity:
            self.stable_count += 1
        else:
            self.stable_count = 1
            self.last_text = text
        return self.stable_count >= self.stable_frames

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
        return
    except queue.Full:
        pass
    try:
        output_queue.get_nowait()
    except Exception:
        pass
    try:
        output_queue.put_nowait(payload)
    except Exception:
        pass


def _ocr_worker_main(request_queue, result_queue, stop_event, processor_kwargs, worker_cpu_set):
    limit_ocr_cpu_threads()
    try:
        from cpu_affinity import apply_current_thread_affinity

        apply_current_thread_affinity(worker_cpu_set, None, "turnsign-ocr-worker")
    except Exception:
        pass

    kwargs = dict(processor_kwargs or {})
    kwargs["log_func"] = None
    processor = None
    try:
        processor = TurnSignOcrApiProcessor(**kwargs)
        # Import and initialize PaddleOCR in the worker process, not in the
        # latency-sensitive control process.  This avoids the observed
        # "partially initialized paddle" state and keeps import spikes off the
        # main loop.
        processor._ensure_ocr()
        _queue_put_latest(result_queue, {"type": "worker_ready", "timestamp": now_seconds()})
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
                _queue_put_latest(result_queue, {"type": "worker_ready", "timestamp": now_seconds()})
            except Exception as exc:
                _queue_put_latest(
                    result_queue,
                    {
                        "type": "result",
                        "timestamp": now_seconds(),
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
        **processor_kwargs,
    ):
        self.log_func = log_func or (lambda _message: None)
        self.worker_cpu_set = str(worker_cpu_set or "").strip()
        self.processor_kwargs = dict(processor_kwargs or {})
        self.processor_kwargs.pop("worker_cpu_set", None)
        self.processor_kwargs.pop("log_func", None)

        self.min_det_score = float(self.processor_kwargs.get("min_det_score", 0.35))
        self.min_area_ratio = float(self.processor_kwargs.get("min_area_ratio", 0.001))
        self.ocr_interval = float(self.processor_kwargs.get("ocr_interval", 0.25))
        self.latest_result = fixed_unknown_result("", "not_started")
        self.latest_response = {
            "active": False,
            "status": "worker_starting",
            "instruction": self.latest_result,
        }
        self.worker_error = ""
        self.worker_ready = False
        self.pending = False
        self.pending_since_ts = 0.0
        # PaddleOCR first inference and the ERNIE/AiStudio API call can both take
        # several seconds on the RK board.  A 4 s timeout was too aggressive and
        # caused the worker to be restarted before the API result could return.
        self.worker_timeout = float(os.environ.get("AR_TURNSIGN_OCR_WORKER_TIMEOUT", "20.0"))
        self.last_submit_ts = 0.0
        self.request_id = 0

        start_method = mp_start_method or os.environ.get("AR_TURNSIGN_OCR_MP_START", "spawn")
        self.ctx = mp.get_context(start_method)
        self._start_worker()

    def _start_worker(self):
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
        self.process_handle.start()
        self.log_func(f"turnsign OCR worker started pid={self.process_handle.pid} cpuset={self.worker_cpu_set or '-'}")

    def _stop_worker(self, terminate=False):
        try:
            if getattr(self, "stop_event", None) is not None:
                self.stop_event.set()
            if getattr(self, "request_queue", None) is not None:
                _queue_put_latest(self.request_queue, {"type": "stop"})
        except Exception:
            pass
        try:
            if self.process_handle is not None:
                self.process_handle.join(timeout=0.8)
                if self.process_handle.is_alive() and terminate:
                    self.process_handle.terminate()
                    self.process_handle.join(timeout=0.5)
        except Exception:
            pass

    def _restart_worker(self, reason):
        self.log_func(f"turnsign OCR worker restart: {reason}")
        self._stop_worker(terminate=True)
        self.pending = False
        self.pending_since_ts = 0.0
        self.worker_ready = False
        self.worker_error = str(reason)
        self._start_worker()

    @staticmethod
    def is_turnsign_detection(det):
        return TurnSignOcrApiProcessor.is_turnsign_detection(det)

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
            candidates.append((score + area * 4.0, det))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def close(self):
        self._stop_worker(terminate=True)

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
                response = dict(item.get("response") or {})
                instruction = response.get("instruction")
                if isinstance(instruction, dict) and bool(response.get("instruction_current", False)):
                    self.latest_result = dict(instruction)
                else:
                    response.setdefault("latest_instruction", self.latest_result)
                self.latest_response = response
                self.pending = False
                self.pending_since_ts = 0.0
                result_received = True
        return result_received

    def process(self, frame, detections, timestamp=None, context=None):
        ts = float(timestamp if timestamp is not None else now_seconds())
        result_received = self._drain_results()

        det = self.select_turnsign(detections)
        if det is None:
            return {
                "active": False,
                "status": "no_turnsign",
                "instruction": fixed_unknown_result("", "no_turnsign"),
                "latest_instruction": self.latest_result,
                "instruction_current": False,
                "worker_ready": self.worker_ready,
                "error": self.worker_error or None,
            }

        base = {
            "active": True,
            "detection": det,
            "instruction": fixed_unknown_result("", "ocr_pending"),
            "latest_instruction": self.latest_result,
            "instruction_current": False,
            "worker_ready": self.worker_ready,
            "error": self.worker_error or None,
        }
        if result_received:
            response = dict(self.latest_response or {})
            response.setdefault("active", True)
            response.setdefault("detection", det)
            response.setdefault("instruction", fixed_unknown_result("", "worker_result"))
            response.setdefault("latest_instruction", self.latest_result)
            response.setdefault("instruction_current", False)
            response["worker_ready"] = self.worker_ready
            if self.worker_error:
                response["error"] = self.worker_error
            return response

        if self.pending:
            if self.worker_timeout > 0.0 and ts - self.pending_since_ts > self.worker_timeout:
                self._restart_worker(f"pending>{self.worker_timeout:.1f}s")
                response = dict(base)
                response["status"] = "ocr_worker_restarted"
                response["error"] = self.worker_error
                return response
            response = dict(base)
            response["status"] = "ocr_pending"
            return response
        if ts - self.last_submit_ts < self.ocr_interval:
            response = dict(base)
            response["active"] = True
            response["detection"] = det
            response["instruction"] = fixed_unknown_result("", "ocr_throttled")
            response["latest_instruction"] = self.latest_result
            response["instruction_current"] = False
            response["status"] = "ocr_throttled"
            response["worker_ready"] = self.worker_ready
            if self.worker_error:
                response["error"] = self.worker_error
            return response

        crop = crop_bbox(frame, det.get("bbox"))
        if crop is None or crop.size == 0:
            response = dict(base)
            response["status"] = "bad_crop"
            return response

        self.request_id += 1
        payload = {
            "type": "ocr",
            "request_id": self.request_id,
            "timestamp": ts,
            "crop": crop,
            "detection": det,
            "context": context or {},
        }
        _queue_put_latest(self.request_queue, payload)
        self.pending = True
        self.pending_since_ts = ts
        self.last_submit_ts = ts
        response = dict(base)
        response["status"] = "ocr_submitted" if self.worker_ready else "worker_starting"
        return response


def load_prompt(args):
    if args.prompt_file:
        with open(args.prompt_file, "r", encoding="utf-8") as f:
            return f.read()
    return args.prompt


def run_text_mode(args):
    interpreter = WenxinRouteInterpreter(
        access_token=args.access_token or args.api_key,
        model=args.model,
        timeout=args.timeout,
        temperature=args.temperature,
        system_prompt=load_prompt(args),
        log_func=print,
    )
    result = interpreter.interpret(args.text, ocr_confidence=args.ocr_confidence)
    print(json.dumps(result, ensure_ascii=False, indent=2))


def run_image_mode(args):
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"failed to read image: {args.image}")
    if args.bbox:
        bbox = [float(v) for v in args.bbox.split(",")]
        image = crop_bbox(image, bbox)
        reader = PaddleOcrReader(
            lang=args.ocr_lang,
            use_angle_cls=not args.no_angle_cls,
            model_root=args.ocr_model_root,
        )
    ocr = reader.read(image)
    print(json.dumps({"ocr": ocr}, ensure_ascii=False, indent=2))
    if args.call_api:
        interpreter = WenxinRouteInterpreter(
            access_token=args.access_token or args.api_key,
            model=args.model,
            timeout=args.timeout,
            temperature=args.temperature,
            system_prompt=load_prompt(args),
            log_func=print,
        )
        result = interpreter.interpret(ocr.get("text", ""), ocr_confidence=ocr.get("confidence", 0.0))
        print(json.dumps({"instruction": result}, ensure_ascii=False, indent=2))


def parse_args():
    parser = argparse.ArgumentParser(description="TurnSign PaddleOCR + ERNIE SDK AI Studio route parser.")
    parser.add_argument("--text", help="Run Wenxin interpretation directly with OCR text.")
    parser.add_argument("--image", help="Run PaddleOCR on an image or cropped TurnSign image.")
    parser.add_argument("--bbox", help="Optional bbox for --image: left,top,right,bottom")
    parser.add_argument("--call-api", action="store_true", help="Call Wenxin API after image OCR.")
    parser.add_argument("--access-token", default=None, help="AI Studio access token. Prefer env AISTUDIO_ACCESS_TOKEN.")
    parser.add_argument("--api-key", default=None, help="Deprecated alias for --access-token.")
    parser.add_argument("--model", default=None, help=f"ERNIE model name, default {DEFAULT_MODEL}.")
    parser.add_argument("--prompt", default=None, help="Override the built-in route parsing prompt.")
    parser.add_argument("--prompt-file", default=None, help="UTF-8 prompt file path.")
    parser.add_argument("--temperature", type=float, default=0.1)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--ocr-confidence", type=float, default=1.0)
    parser.add_argument("--ocr-lang", default="ch")
    parser.add_argument("--no-angle-cls", action="store_true")
    parser.add_argument("--ocr-model-root", default=None, help="ASCII path containing PaddleOCR det/rec/cls model dirs.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.text:
        run_text_mode(args)
        return
    if args.image:
        run_image_mode(args)
        return
    raise SystemExit("use --text or --image")


if __name__ == "__main__":
    main()

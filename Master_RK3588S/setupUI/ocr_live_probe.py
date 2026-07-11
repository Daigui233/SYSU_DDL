#!/usr/bin/env python3
import argparse
import json
import os
import time
from multiprocessing import shared_memory

import cv2

from turnsign_ocr_api import (
    PaddleOcrReader,
    WenxinRouteInterpreter,
    crop_bbox,
    fixed_unknown_result,
    load_local_ocr_env,
    repair_python_logging_levels,
)
from vision_frame_source import FrameSnapshotChanged, read_frame_from_shm, remove_shm_from_resource_tracker


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16
DEFAULT_MODEL_DIR = os.path.join(BASE_DIR, "infer_wrap", "base", "model")
CLASSES = ()


def parse_bbox(text):
    if not text:
        return None
    parts = [item.strip() for item in str(text).split(",")]
    if len(parts) != 4:
        raise ValueError("bbox must be left,top,right,bottom")
    return [float(item) for item in parts]


def write_json(path, payload):
    if not path:
        return
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
    os.replace(tmp_path, path)


def class_name(class_id):
    idx = int(class_id)
    if 0 <= idx < len(CLASSES):
        return CLASSES[idx]
    return f"cls_{idx}"


def load_detector(model_dir, core_id):
    global CLASSES
    # Keep PaddleOCR import/initialization before RKNNLite import.  RKNNLite can
    # corrupt stdlib logging's level-name table; repair it again after import so
    # API clients and future Paddle calls remain healthy.
    from infer_wrap import InferWrap
    from infer_wrap.base.func import CLASSES as DET_CLASSES

    repair_python_logging_levels()
    CLASSES = tuple(DET_CLASSES)
    return InferWrap(model_dir=model_dir, TPEs=1, core_ids=[core_id], max_inflight=1)


def is_turnsign(label):
    key = str(label or "").strip().lower().replace("_", "").replace("-", "").replace(" ", "")
    return key == "turnsign"


def build_detections(det_res, frame):
    if det_res is None:
        return []
    try:
        boxes, scores, classes = det_res
    except Exception:
        return []
    if boxes is None or scores is None or classes is None:
        return []
    h, w = frame.shape[:2]
    detections = []
    for box, score, class_id in zip(boxes, scores, classes):
        try:
            left, top, right, bottom = [float(v) for v in box]
            score_f = float(score)
            class_i = int(class_id)
        except Exception:
            continue
        left = max(0.0, min(float(w - 1), left))
        right = max(0.0, min(float(w - 1), right))
        top = max(0.0, min(float(h - 1), top))
        bottom = max(0.0, min(float(h - 1), bottom))
        if right <= left or bottom <= top:
            continue
        label = class_name(class_i)
        area = (right - left) * (bottom - top)
        detections.append(
            {
                "class_id": class_i,
                "label": label,
                "score": score_f,
                "bbox": [left, top, right, bottom],
                "area_ratio": area / float(max(1, h * w)),
            }
        )
    detections.sort(key=lambda item: item["score"], reverse=True)
    return detections


def select_turnsign(detections, min_score, min_area_ratio):
    candidates = [
        det for det in detections
        if is_turnsign(det.get("label"))
        and float(det.get("score") or 0.0) >= float(min_score)
        and float(det.get("area_ratio") or 0.0) >= float(min_area_ratio)
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda det: float(det.get("score") or 0.0) + float(det.get("area_ratio") or 0.0) * 4.0, reverse=True)
    return candidates[0]


def run_once(shm, detector, reader, interpreter, args):
    fid, frame = read_frame_from_shm(shm, args.header_size)
    manual_bbox = parse_bbox(args.bbox)

    det_started = time.perf_counter()
    det_res = None
    det_flag = False
    det_meta = {}
    detections = []
    turnsign_det = None
    if manual_bbox is None:
        det_res, det_flag, det_meta = detector.infer_current(frame, frame_id=int(fid), timestamp=time.time())
        detections = build_detections(det_res, frame) if det_flag else []
        turnsign_det = select_turnsign(detections, args.min_det_score, args.min_area_ratio)
        bbox = None if turnsign_det is None else turnsign_det.get("bbox")
    else:
        bbox = manual_bbox
        turnsign_det = {
            "label": "TurnSign",
            "score": 1.0,
            "bbox": bbox,
            "area_ratio": 0.0,
            "manual": True,
        }
    det_ms = (time.perf_counter() - det_started) * 1000.0
    det_npu_ms = float(det_meta.get("npu_inference_ms") or 0.0) if isinstance(det_meta, dict) else 0.0

    if turnsign_det is None:
        result = {
            "ok": True,
            "timestamp": time.time(),
            "fid": int(fid),
            "status": "no_turnsign",
            "det_ms": round(det_ms, 2),
            "det_npu_ms": round(det_npu_ms, 2),
            "api_enabled": bool(interpreter.enabled()),
            "detections": detections[:8],
            "ocr": {"text": "", "confidence": 0.0, "lines": []},
            "instruction": fixed_unknown_result("", "no_turnsign"),
        }
        return result

    crop = crop_bbox(frame, bbox, pad_ratio=args.pad_ratio)
    if crop is None or crop.size == 0:
        raise RuntimeError("empty OCR crop")
    if args.scale and abs(float(args.scale) - 1.0) > 1e-3:
        crop = cv2.resize(
            crop,
            None,
            fx=float(args.scale),
            fy=float(args.scale),
            interpolation=cv2.INTER_CUBIC,
        )

    if args.crop_out:
        cv2.imwrite(args.crop_out, crop)

    ocr_started = time.perf_counter()
    ocr = reader.read(crop)
    ocr_ms = (time.perf_counter() - ocr_started) * 1000.0

    instruction = fixed_unknown_result(ocr.get("text", ""), "api_not_called")
    api_ms = 0.0
    if not args.no_api and ocr.get("text"):
        api_started = time.perf_counter()
        instruction = interpreter.interpret(
            ocr.get("text", ""),
            ocr_confidence=ocr.get("confidence", 0.0),
            context={
                "mode": "ocr_live_probe",
                "fid": int(fid),
                "bbox": bbox,
                "frame_shape": [int(v) for v in frame.shape[:2]],
                "crop_shape": [int(v) for v in crop.shape[:2]],
            },
        )
        api_ms = (time.perf_counter() - api_started) * 1000.0

    return {
        "ok": True,
        "timestamp": time.time(),
        "fid": int(fid),
        "status": "turnsign_ocr_done",
        "det_ms": round(det_ms, 2),
        "det_npu_ms": round(det_npu_ms, 2),
        "turnsign": turnsign_det,
        "detections": detections[:8],
        "bbox": bbox,
        "frame_shape": [int(v) for v in frame.shape[:2]],
        "crop_shape": [int(v) for v in crop.shape[:2]],
        "ocr_ms": round(ocr_ms, 2),
        "api_ms": round(api_ms, 2),
        "api_enabled": bool(interpreter.enabled()),
        "ocr": ocr,
        "instruction": instruction,
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live TurnSign OCR/API probe. Reads camera shared memory only; no RKNN, no line following, no car control."
    )
    parser.add_argument("--shm-name", default=SHM_NAME)
    parser.add_argument("--header-size", type=int, default=SHM_HEADER_SIZE)
    parser.add_argument("--model-dir", default=os.environ.get("AR_MODEL_DIR", DEFAULT_MODEL_DIR))
    parser.add_argument("--det-core", type=int, default=int(os.environ.get("AR_OCR_DET_CORE", "0")))
    parser.add_argument("--min-det-score", type=float, default=0.30)
    parser.add_argument("--min-area-ratio", type=float, default=0.0005)
    parser.add_argument("--bbox", default=os.environ.get("AR_OCR_BBOX", ""), help="debug override only: left,top,right,bottom; default uses detector TurnSign bbox")
    parser.add_argument("--scale", type=float, default=float(os.environ.get("AR_OCR_SCALE", "1.0")))
    parser.add_argument("--pad-ratio", type=float, default=0.12)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--count", type=int, default=1, help="0 means run forever")
    parser.add_argument("--no-api", action="store_true", help="only OCR; skip AI Studio interpretation")
    parser.add_argument("--crop-out", default="", help="optional path to save latest OCR crop")
    parser.add_argument("--output-json", default=os.path.join(BASE_DIR, "ocr_live_status.json"))
    parser.add_argument("--pretty", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    load_local_ocr_env(BASE_DIR)
    detector = None
    print("[ocr-live] initializing PaddleOCR ...")
    reader = PaddleOcrReader()
    interpreter = WenxinRouteInterpreter(log_func=lambda message: print(f"[ocr-live-api] {message}"))
    if args.bbox:
        print("[ocr-live] manual bbox debug mode: detector is skipped")
    else:
        print("[ocr-live] initializing TurnSign detector ...")
        detector = load_detector(args.model_dir, args.det_core)
    source_text = f"manual_bbox={args.bbox}" if args.bbox else "detector=TurnSign"
    print(f"[ocr-live] api_enabled={interpreter.enabled()} {source_text} scale={args.scale}")

    completed = 0
    while True:
        try:
            shm = shared_memory.SharedMemory(name=args.shm_name)
            remove_shm_from_resource_tracker(args.shm_name)
            break
        except FileNotFoundError:
            print(f"[ocr-live] waiting for shared memory {args.shm_name} ...")
            time.sleep(1.0)

    try:
        while args.count <= 0 or completed < args.count:
            try:
                result = run_once(shm, detector, reader, interpreter, args)
            except FrameSnapshotChanged:
                continue
            except Exception as exc:
                result = {
                    "ok": False,
                    "timestamp": time.time(),
                    "error": str(exc),
                }
            write_json(args.output_json, result)
            text = (result.get("ocr") or {}).get("text", "")
            conf = (result.get("ocr") or {}).get("confidence", 0.0)
            instr = result.get("instruction") or {}
            print(
                json.dumps(result, ensure_ascii=False, indent=2)
                if args.pretty
                else (
                    f"[ocr-live] ok={result.get('ok')} status={result.get('status')} fid={result.get('fid')} "
                    f"det_ms={result.get('det_ms')} npu={result.get('det_npu_ms')} "
                    f"ocr_ms={result.get('ocr_ms')} api_ms={result.get('api_ms')} "
                    f"text={text!r} conf={float(conf or 0):.2f} "
                    f"valid={instr.get('valid')} action={instr.get('action')} "
                    f"pref={instr.get('preferred_branch')} reason={instr.get('reason')}"
                )
            )
            completed += 1
            if args.count > 0 and completed >= args.count:
                break
            time.sleep(max(0.0, float(args.interval)))
    finally:
        try:
            shm.close()
        except Exception:
            pass
        try:
            detector.release()
        except Exception:
            pass


if __name__ == "__main__":
    main()

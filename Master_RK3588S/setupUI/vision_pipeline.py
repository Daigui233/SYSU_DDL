import os
import time

import cv2

from infer_wrap import InferWrap, PPSegInfer
from infer_wrap.base.func import CLASSES
from infer_wrap.base.seg_func import draw_centerline_debug, extract_centerline_info
try:
    from infer_wrap.base.seg_infer import ForkMaskClsInfer
except Exception:
    ForkMaskClsInfer = None
from vision_traffic_light import classify_traffic_light_color

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_DIR = os.environ.get("AR_MODEL_DIR", os.path.join(BASE_DIR, "infer_wrap", "base", "model"))


def draw_waiting(frame):
    cv2.putText(frame, "Road seg: waiting", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Midline: waiting", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Track err: N/A", (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)


class VisionPipeline:
    """Owns NPU inference and converts model outputs into overlays and perception."""

    def __init__(self, model_dir=None, seg_result_ttl=None, det_result_ttl=None, log_func=None):
        self.model_dir = model_dir or DEFAULT_MODEL_DIR
        # Constructor compatibility only: segmentation and detection results must be current-frame.
        _ = seg_result_ttl
        _ = det_result_ttl
        self.seg_result_ttl = 0.0
        self.det_result_ttl = 0.0
        self.log_func = log_func or print

        self.infer_det = None
        self.infer_seg = None
        self.infer_fork = None
        self._status = {
            "ok": False,
            "detector": "not initialized",
            "segmenter": "not initialized",
            "fork_classifier": "not initialized",
            "error": None,
        }
        self._init_engines()

    def _log(self, message):
        self.log_func(message)

    def _init_engines(self):
        self._log("initializing NPU AI...")
        try:
            self.infer_det = InferWrap(model_dir=self.model_dir, TPEs=1, core_ids=[0], max_inflight=1)
            self.infer_seg = PPSegInfer(model_dir=self.model_dir, TPEs=1, core_ids=[1], max_inflight=1)
            if ForkMaskClsInfer is not None:
                try:
                    self.infer_fork = ForkMaskClsInfer(model_dir=self.model_dir, core_id=2)
                except FileNotFoundError as exc:
                    self.infer_fork = None
                    self._log(f"fork classifier disabled: {exc}")
        except Exception as exc:
            self.infer_det = None
            self.infer_seg = None
            self.infer_fork = None
            self._status = {
                "ok": False,
                "detector": "disabled",
                "segmenter": "disabled",
                "fork_classifier": "disabled",
                "error": str(exc),
            }
            self._log(f"NPU AI init failed: {exc}; WebUI and pose forwarding continue without vision inference")
            return

        self._status = {
            "ok": True,
            "detector": "ready",
            "segmenter": "ready",
            "fork_classifier": "ready" if self.infer_fork is not None else "not found",
            "error": None,
        }
        self._log("AI engines loaded")

    def status(self):
        return dict(self._status)

    @staticmethod
    def _normalize_frame_id(frame_id):
        if frame_id is None:
            return None
        try:
            return int(frame_id)
        except Exception:
            return None

    @staticmethod
    def _empty_segmentation(frame, now, source="missing", frame_id=None):
        h, w = frame.shape[:2]
        frame_id_value = VisionPipeline._normalize_frame_id(frame_id)
        return {
            "timestamp": float(now),
            "age": None,
            "frame_id": frame_id_value,
            "seg_frame_id": None,
            "seg_lag_frames": None,
            "seg_age_ms": None,
            "source": source,
            "line_valid": False,
            "track_error": None,
            "road_valid": False,
            "road_held": False,
            "road_state": "LOST",
            "midline_valid": False,
            "midline_state": "LOST",
            "road_ratio": 0.0,
            "reason": source,
            "branch_count": 0,
            "center_x": w // 2,
            "target_x": None,
            "road_mask": None,
            "mid_points": [],
            "fork_state": "MISS",
            "fork_mode": "single",
            "channel_state": "TRACK",
            "channel_hold": 0,
            "postproc_state": "OFF",
            "fork_strength": 0.0,
            "fork_classifier": {"available": False, "name": "none", "confidence": 0.0},
            "fork_geometry_valid": False,
            "fork_split_rows": 0,
            "fork_left_pixels": 0,
            "fork_right_pixels": 0,
            "fork_total_pixels": 0,
            "fork_branch_pixel_threshold": 0,
            "fork_total_pixel_threshold": 0,
            "fork_candidates": [],
            "fork_selected_side": None,
            "fork_reason": source,
        }

    @staticmethod
    def _empty_detection_status(now, source="missing", frame_id=None, reason=None):
        frame_id_value = VisionPipeline._normalize_frame_id(frame_id)
        return {
            "timestamp": float(now),
            "source": source,
            "reason": reason or source,
            "frame_id": frame_id_value,
            "det_frame_id": None,
            "det_lag_frames": None,
            "det_age_ms": None,
            "count": 0,
        }

    @staticmethod
    def _class_name(class_id):
        idx = int(class_id)
        if 0 <= idx < len(CLASSES):
            return CLASSES[idx]
        return f"cls_{idx}"

    @staticmethod
    def _category_for_label(label):
        key = str(label).strip().lower()
        aliases = {
            "coin": "gold",
            "gold": "gold",
            "human": "human",
            "person": "human",
            "car": "car",
            "door": "door",
            "stone": "stone",
            "rock": "stone",
            "trafficlight": "traffic_light",
            "traffic_light": "traffic_light",
            "beginsign": "begin_sign",
            "begin_sign": "begin_sign",
            "endsign": "end_sign",
            "end_sign": "end_sign",
        }
        return aliases.get(key, key)

    def _build_detection_list(self, det_res, frame, timestamp, age, frame_id=None, det_frame_id=None):
        if det_res is None:
            return []

        try:
            boxes, scores, classes = det_res
        except Exception:
            return []

        if boxes is None or scores is None or classes is None:
            return []

        h, w = frame.shape[:2]
        frame_id_value = self._normalize_frame_id(frame_id)
        det_frame_id_value = self._normalize_frame_id(det_frame_id)
        det_lag_frames = None
        if frame_id_value is not None and det_frame_id_value is not None:
            det_lag_frames = frame_id_value - det_frame_id_value
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

            label = self._class_name(class_i)
            category = self._category_for_label(label)
            box_w = right - left
            box_h = bottom - top
            area = box_w * box_h
            det = {
                "timestamp": float(timestamp),
                "age": float(age),
                "frame_id": frame_id_value,
                "det_frame_id": det_frame_id_value,
                "det_lag_frames": det_lag_frames,
                "det_age_ms": float(age * 1000.0),
                "class_id": class_i,
                "label": label,
                "category": category,
                "score": score_f,
                "bbox": [left, top, right, bottom],
                "center": [left + box_w * 0.5, top + box_h * 0.5],
                "size": [box_w, box_h],
                "area_ratio": area / float(max(1, h * w)),
            }
            if category == "traffic_light":
                det.update(classify_traffic_light_color(frame, det["bbox"]))
            detections.append(det)

        detections.sort(key=lambda item: item["score"], reverse=True)
        return detections

    @staticmethod
    def _draw_detection_extras(frame, detections):
        for det in detections:
            if det.get("category") != "traffic_light":
                continue
            state = str(det.get("traffic_light_state") or "unknown")
            conf = float(det.get("traffic_light_confidence") or 0.0)
            bbox = det.get("bbox") or [0, 0, 0, 0]
            left, top = int(bbox[0]), int(bbox[1])
            color = (0, 255, 255)
            if state == "red":
                color = (0, 0, 255)
            elif state == "green":
                color = (0, 255, 0)
            cv2.putText(
                frame,
                f"TL {state} {conf:.2f}",
                (left, max(42, top + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

    @staticmethod
    def _draw_detection_list(frame, detections):
        if not detections:
            return frame
        h, w = frame.shape[:2]
        for det in detections:
            bbox = det.get("bbox") or [0, 0, 0, 0]
            left, top, right, bottom = [int(v) for v in bbox]
            left = max(0, min(w - 1, left))
            right = max(0, min(w - 1, right))
            top = max(0, min(h - 1, top))
            bottom = max(0, min(h - 1, bottom))
            if right <= left or bottom <= top:
                continue
            label = det.get("label") or det.get("category") or "obj"
            score = float(det.get("score") or 0.0)
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{label} {score:.2f}",
                (left, max(20, top - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        return frame

    def draw_debug_frame(self, frame, perception):
        out = frame.copy()
        perception = perception or {}
        segmentation = perception.get("segmentation") or {}
        if segmentation.get("source") == "current" or segmentation.get("road_mask") is not None:
            out = draw_centerline_debug(out, segmentation, draw_text=False)

        detections = list(perception.get("detections") or [])
        self._draw_detection_list(out, detections)
        self._draw_detection_extras(out, detections)
        return out

    def process(self, frame, now, frame_id=None, draw_debug=False):
        final_frame = frame.copy() if draw_debug else frame
        frame_id_value = self._normalize_frame_id(frame_id)
        segmentation = self._empty_segmentation(frame, now, frame_id=frame_id)
        detection_status = self._empty_detection_status(
            now,
            source="disabled" if self.infer_det is None else "missing",
            frame_id=frame_id_value,
        )
        detections = []

        if self.infer_seg is not None:
            try:
                if hasattr(self.infer_seg, "infer_current"):
                    seg_res, seg_flag, seg_meta = self.infer_seg.infer_current(
                        frame.copy(),
                        frame_id=frame_id,
                        timestamp=now,
                    )
                else:
                    raise RuntimeError("segmenter does not support current-frame inference")

                seg_frame_id = self._normalize_frame_id(seg_meta.get("frame_id"))
                frame_matches = frame_id_value is None or seg_frame_id == frame_id_value
                if seg_flag and seg_res is not None and frame_matches:
                    final_frame, segmentation = extract_centerline_info(
                        seg_res,
                        final_frame,
                        fork_classifier=self.infer_fork,
                        draw_debug=draw_debug,
                    )
                    seg_done_ts = time.time()
                    seg_age = max(0.0, seg_done_ts - float(seg_meta.get("timestamp") or now))
                    segmentation["timestamp"] = float(now)
                    segmentation["age"] = seg_age
                    segmentation["frame_id"] = frame_id_value
                    segmentation["seg_frame_id"] = seg_frame_id
                    segmentation["seg_lag_frames"] = 0 if frame_id_value is not None and seg_frame_id is not None else None
                    segmentation["seg_age_ms"] = float(seg_age * 1000.0)
                    segmentation["source"] = "current"
                else:
                    if seg_flag and seg_res is not None and not frame_matches:
                        segmentation = self._empty_segmentation(frame, now, source="stale_rejected", frame_id=frame_id)
                    if draw_debug:
                        draw_waiting(final_frame)
            except Exception as exc:
                self._log(f"seg infer skip: {exc}")
                segmentation = self._empty_segmentation(frame, now, source="seg_error", frame_id=frame_id)
                if draw_debug:
                    draw_waiting(final_frame)
        else:
            if draw_debug:
                draw_waiting(final_frame)

        if self.infer_det is not None:
            try:
                if hasattr(self.infer_det, "infer_current"):
                    det_res, det_flag, det_meta = self.infer_det.infer_current(
                        frame.copy(),
                        frame_id=frame_id,
                        timestamp=now,
                    )
                else:
                    raise RuntimeError("detector does not support current-frame inference")

                det_frame_id = self._normalize_frame_id((det_meta or {}).get("frame_id"))
                det_done_ts = time.time()
                det_age = max(0.0, det_done_ts - float((det_meta or {}).get("timestamp") or now))
                det_matches = frame_id_value is None or det_frame_id == frame_id_value
                detection_status = {
                    "timestamp": float(now),
                    "source": "current" if det_flag and det_res is not None and det_matches else "missing",
                    "reason": "current" if det_flag and det_res is not None and det_matches else "no_result",
                    "frame_id": frame_id_value,
                    "det_frame_id": det_frame_id,
                    "det_lag_frames": (
                        frame_id_value - det_frame_id
                        if frame_id_value is not None and det_frame_id is not None
                        else None
                    ),
                    "det_age_ms": float(det_age * 1000.0),
                    "count": 0,
                }
                if det_flag and det_res is not None and det_matches:
                    detections = self._build_detection_list(
                        det_res,
                        frame,
                        now,
                        det_age,
                        frame_id=frame_id_value,
                        det_frame_id=det_frame_id,
                    )
                    detection_status["count"] = len(detections)
                    if draw_debug:
                        self._draw_detection_list(final_frame, detections)
                        self._draw_detection_extras(final_frame, detections)
                elif det_flag and det_res is not None and not det_matches:
                    detection_status["source"] = "stale_rejected"
                    detection_status["reason"] = "frame_id_mismatch"
            except Exception as exc:
                self._log(f"det infer skip: {exc}")
                detection_status = self._empty_detection_status(
                    now,
                    source="det_error",
                    frame_id=frame_id_value,
                    reason=str(exc),
                )

        perception = {
            "timestamp": float(now),
            "frame_shape": [int(v) for v in frame.shape[:2]],
            "segmentation": segmentation,
            "detection_status": detection_status,
            "detections": detections,
        }
        return final_frame, perception

    def release(self):
        if self.infer_seg is not None:
            self.infer_seg.release()
        if self.infer_det is not None:
            self.infer_det.release()
        if self.infer_fork is not None:
            self.infer_fork.release()

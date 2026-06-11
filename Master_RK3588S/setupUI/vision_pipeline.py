import os

import cv2

from infer_wrap import InferWrap, PPSegInfer
from infer_wrap.base.func import CLASSES, draw
from infer_wrap.base.seg_func import extract_centerline_info
try:
    from infer_wrap.base.seg_infer import ForkMaskClsInfer
except Exception:
    ForkMaskClsInfer = None
from vision_traffic_light import classify_traffic_light_color

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_DIR = os.environ.get("AR_MODEL_DIR", os.path.join(BASE_DIR, "infer_wrap", "base", "model"))
DEFAULT_SEG_RESULT_TTL = float(os.environ.get("AR_SEG_RESULT_TTL", "2.0"))
DEFAULT_DET_RESULT_TTL = float(os.environ.get("AR_DET_RESULT_TTL", "1.0"))


def draw_waiting(frame):
    cv2.putText(frame, "Road seg: waiting", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Midline: waiting", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Track err: N/A", (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)


class VisionPipeline:
    """Owns NPU inference and converts model outputs into overlays and perception."""

    def __init__(self, model_dir=None, seg_result_ttl=None, det_result_ttl=None, log_func=None):
        self.model_dir = model_dir or DEFAULT_MODEL_DIR
        self.seg_result_ttl = float(DEFAULT_SEG_RESULT_TTL if seg_result_ttl is None else seg_result_ttl)
        self.det_result_ttl = float(DEFAULT_DET_RESULT_TTL if det_result_ttl is None else det_result_ttl)
        self.log_func = log_func or print

        self.infer_det = None
        self.infer_seg = None
        self.infer_fork = None
        self.last_seg_res = None
        self.last_seg_ts = 0.0
        self.last_det_res = None
        self.last_det_ts = 0.0
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
    def _empty_segmentation(frame, now, source="missing"):
        h, w = frame.shape[:2]
        return {
            "timestamp": float(now),
            "age": None,
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
            "fork_classifier": {"available": False, "name": "none", "confidence": 0.0},
            "fork_geometry_valid": False,
            "fork_split_rows": 0,
            "fork_candidates": [],
            "fork_selected_side": None,
            "fork_reason": source,
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

    def _build_detection_list(self, det_res, frame, timestamp, age):
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

            label = self._class_name(class_i)
            category = self._category_for_label(label)
            box_w = right - left
            box_h = bottom - top
            area = box_w * box_h
            det = {
                "timestamp": float(timestamp),
                "age": float(age),
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

    def process(self, frame, now):
        final_frame = frame.copy()
        segmentation = self._empty_segmentation(frame, now)
        detections = []

        if self.infer_seg is not None:
            try:
                seg_res, seg_flag = self.infer_seg.infer(frame.copy())
                if seg_flag and seg_res is not None:
                    self.last_seg_res = seg_res
                    self.last_seg_ts = now
            except Exception as exc:
                self._log(f"seg infer skip: {exc}")

        if self.infer_det is not None:
            try:
                det_res, det_flag = self.infer_det.infer(frame.copy())
                if det_flag and det_res is not None:
                    self.last_det_res = det_res
                    self.last_det_ts = now
            except Exception as exc:
                self._log(f"det infer skip: {exc}")

        if self.last_seg_res is not None and (now - self.last_seg_ts) <= self.seg_result_ttl:
            try:
                final_frame, segmentation = extract_centerline_info(
                    self.last_seg_res,
                    final_frame,
                    fork_classifier=self.infer_fork,
                )
                segmentation["timestamp"] = float(self.last_seg_ts)
                segmentation["age"] = float(now - self.last_seg_ts)
                segmentation["source"] = "fresh" if segmentation["age"] <= 0.05 else "cached"
            except Exception as exc:
                self._log(f"seg draw skip: {exc}")
                draw_waiting(final_frame)
        else:
            draw_waiting(final_frame)

        if self.last_det_res is not None and (now - self.last_det_ts) <= self.det_result_ttl:
            try:
                boxes, scores, classes = self.last_det_res
                draw(final_frame, boxes, scores, classes)
                detections = self._build_detection_list(
                    self.last_det_res,
                    final_frame,
                    self.last_det_ts,
                    now - self.last_det_ts,
                )
                self._draw_detection_extras(final_frame, detections)
            except Exception as exc:
                self._log(f"det draw skip: {exc}")

        perception = {
            "timestamp": float(now),
            "frame_shape": [int(v) for v in frame.shape[:2]],
            "segmentation": segmentation,
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

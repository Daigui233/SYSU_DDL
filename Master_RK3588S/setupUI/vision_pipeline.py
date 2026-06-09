import cv2

from infer_wrap import InferWrap, PPSegInfer
from infer_wrap.base.func import draw
from infer_wrap.base.seg_func import extract_centerline


def draw_waiting(frame):
    cv2.putText(frame, "Road seg: waiting", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Midline: waiting", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Track err: N/A", (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)


class VisionPipeline:
    """Owns NPU model inference and converts model outputs into frame overlays and track_error."""

    def __init__(self, model_dir, seg_result_ttl=2.0, det_result_ttl=1.0, log_func=None):
        self.model_dir = model_dir
        self.seg_result_ttl = float(seg_result_ttl)
        self.det_result_ttl = float(det_result_ttl)
        self.log_func = log_func or print

        self.infer_det = None
        self.infer_seg = None
        self.last_seg_res = None
        self.last_seg_ts = 0.0
        self.last_det_res = None
        self.last_det_ts = 0.0
        self._status = {
            "ok": False,
            "detector": "not initialized",
            "segmenter": "not initialized",
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
        except Exception as exc:
            self.infer_det = None
            self.infer_seg = None
            self._status = {
                "ok": False,
                "detector": "disabled",
                "segmenter": "disabled",
                "error": str(exc),
            }
            self._log(f"NPU AI init failed: {exc}; WebUI and pose forwarding continue without vision inference")
            return

        self._status = {
            "ok": True,
            "detector": "ready",
            "segmenter": "ready",
            "error": None,
        }
        self._log("AI engines loaded")

    def status(self):
        return dict(self._status)

    def process(self, frame, now):
        final_frame = frame.copy()
        track_error = None

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
                final_frame, track_error = extract_centerline(self.last_seg_res, final_frame)
            except Exception as exc:
                self._log(f"seg draw skip: {exc}")
                draw_waiting(final_frame)
        else:
            draw_waiting(final_frame)

        if self.last_det_res is not None and (now - self.last_det_ts) <= self.det_result_ttl:
            try:
                boxes, scores, classes = self.last_det_res
                draw(final_frame, boxes, scores, classes)
            except Exception as exc:
                self._log(f"det draw skip: {exc}")

        return final_frame, track_error

    def release(self):
        if self.infer_seg is not None:
            self.infer_seg.release()
        if self.infer_det is not None:
            self.infer_det.release()

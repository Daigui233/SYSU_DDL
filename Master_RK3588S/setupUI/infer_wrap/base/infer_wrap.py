import os
from pathlib import Path

import numpy as np

from .func import finalize_inference, inference_worker


def get_current_dir():
    return Path(__file__).resolve().parent


class InferWrap:
    """Three-core pipelined wrapper for the multi-task RKNN model."""

    def __init__(self, model_dir="model", TPEs=3, model_path=None):
        # Keep rknnlite board-only: NumPy post-processing can still be imported
        # and tested on a development PC without the RKNN runtime installed.
        from .rknnpool import rknnPoolExecutor

        model_dir = get_current_dir() / model_dir
        selected_model = self.get_model_path(model_dir, model_path)
        self.model_path = str(selected_model)
        # Parallel mode keeps one runtime per NPU core for throughput. The
        # all-cores mode is available for final-model latency comparison.
        self.npu_mode = os.environ.get(
            "MULTITASK_NPU_MODE", "parallel").strip().lower()
        if self.npu_mode not in {"parallel", "all_cores"}:
            raise ValueError(
                "MULTITASK_NPU_MODE must be parallel or all_cores")
        use_all_cores = self.npu_mode == "all_cores"
        self.TPEs = 1 if use_all_cores else max(1, min(int(TPEs), 3))
        # Keep every NPU worker busy. Depth 1 serializes put/get and drops the
        # three-core setup to single-inference throughput; matching the queue
        # depth to the worker count restores the intended real-time FPS.
        default_depth = self.TPEs
        try:
            requested_depth = int(os.environ.get(
                "MULTITASK_PIPELINE_DEPTH", str(default_depth)))
        except ValueError:
            requested_depth = default_depth
        self.pipeline_depth = max(1, min(requested_depth, self.TPEs))
        self.rknn_pool = rknnPoolExecutor(
            rknnModel=self.model_path,
            TPEs=self.TPEs,
            func=inference_worker,
            use_all_cores=use_all_cores,
        )
        self.warmup_inference_ms = []
        warmup_value = os.environ.get("MULTITASK_RKNN_WARMUP", "1")
        warmup_enabled = warmup_value.strip().lower() not in {
            "0", "false", "no", "off"}
        if warmup_enabled:
            try:
                self._warmup_runtimes()
            except Exception:
                self.rknn_pool.release()
                raise
        self.pending = 0

    def _warmup_runtimes(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for runtime in self.rknn_pool.rknnPool:
            worker_result = inference_worker(runtime, frame)
            decoded = finalize_inference(worker_result)
            self.warmup_inference_ms.append(float(
                decoded["timings_ms"]["inference"]))

    @staticmethod
    def get_model_path(model_dir, model_path=None):
        explicit = model_path or os.environ.get("MULTITASK_RKNN_MODEL", "")
        if explicit:
            path = Path(explicit).expanduser().resolve()
            if not path.is_file():
                raise FileNotFoundError("configured RKNN model not found: {}".format(path))
            return path

        model_dir = Path(model_dir)
        variant = os.environ.get("MULTITASK_RKNN_VARIANT", "curve_best").strip().lower()
        variant_files = {
            "int8": "multitask_ppyoloe_int8.rknn",
            "fp16": "multitask_ppyoloe_fp16.rknn",
            "curve_best": "multitask_ppyoloe_curve_best_int8.rknn",
            "multitask_best": "multitask_ppyoloe_multitask_best_int8.rknn",
        }
        if variant not in variant_files:
            raise ValueError(
                "MULTITASK_RKNN_VARIANT must be one of: {}".format(
                    ", ".join(sorted(variant_files))))
        preferred = model_dir / variant_files[variant]
        if preferred.is_file():
            return preferred.resolve()
        available = sorted(model_dir.glob("multitask_ppyoloe_*.rknn"))
        if len(available) == 1:
            return available[0].resolve()
        raise FileNotFoundError(
            "missing {}; place the exported model in {} or set "
            "MULTITASK_RKNN_MODEL. Available multitask models: {}".format(
                preferred.name, model_dir,
                ", ".join(item.name for item in available) or "none"))

    def infer(self, img):
        self.rknn_pool.put(img)
        self.pending += 1
        if self.pending < self.pipeline_depth:
            return None, False
        result, ready = self.rknn_pool.get()
        self.pending -= 1
        if ready:
            result = finalize_inference(result)
        return result, ready

    def __call__(self, *args, **kwargs):
        return self.infer(*args, **kwargs)

    def release(self):
        self.rknn_pool.release()


if __name__ == "__main__":
    import cv2
    from .func import render_result

    infer = InferWrap("model", TPEs=1)
    image = cv2.imread("./bus.jpg")
    result, ready = infer.infer(image)
    if ready and result:
        rendered = render_result(result["frame"].copy(), result, mode="full")
        cv2.imwrite("result.jpg", rendered)
    infer.release()

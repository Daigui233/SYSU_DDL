import os
from pathlib import Path

from .func import myFunc


def get_current_dir():
    return Path(__file__).resolve().parent


class InferWrap:
    """Threaded single-RKNN multi-task inference wrapper."""

    def __init__(self, model_dir="model", TPEs=1, model_path=None):
        # Keep rknnlite board-only: NumPy post-processing can still be imported
        # and tested on a development PC without the RKNN runtime installed.
        from .rknnpool import rknnPoolExecutor

        model_dir = get_current_dir() / model_dir
        selected_model = self.get_model_path(model_dir, model_path)
        self.model_path = str(selected_model)
        self.TPEs = max(1, int(TPEs))
        try:
            requested_depth = int(os.environ.get("MULTITASK_PIPELINE_DEPTH", "1"))
        except ValueError:
            requested_depth = 1
        self.pipeline_depth = max(1, min(requested_depth, self.TPEs))
        self.rknn_pool = rknnPoolExecutor(
            rknnModel=self.model_path, TPEs=self.TPEs, func=myFunc)
        self.pending = 0

    @staticmethod
    def get_model_path(model_dir, model_path=None):
        explicit = model_path or os.environ.get("MULTITASK_RKNN_MODEL", "")
        if explicit:
            path = Path(explicit).expanduser().resolve()
            if not path.is_file():
                raise FileNotFoundError("configured RKNN model not found: {}".format(path))
            return path

        model_dir = Path(model_dir)
        variant = os.environ.get("MULTITASK_RKNN_VARIANT", "fp16").strip().lower()
        if variant not in {"fp16", "int8"}:
            raise ValueError("MULTITASK_RKNN_VARIANT must be fp16 or int8")
        preferred = model_dir / "multitask_ppyoloe_{}.rknn".format(variant)
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
        result = self.rknn_pool.get()
        self.pending -= 1
        return result

    def __call__(self, *args, **kwargs):
        return self.infer(*args, **kwargs)

    def release(self):
        self.rknn_pool.release()


if __name__ == "__main__":
    import cv2

    infer = InferWrap("model", TPEs=1)
    image = cv2.imread("./bus.jpg")
    result, ready = infer.infer(image)
    if ready and result:
        cv2.imwrite("result.jpg", result["frame"])
    infer.release()

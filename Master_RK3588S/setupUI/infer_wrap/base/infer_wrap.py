import glob
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

try:
    from .func import myFunc
    from .rknnpool import rknnPoolExecutor
except ImportError:
    from func import myFunc
    from rknnpool import rknnPoolExecutor


def get_current_dir():
    return os.path.dirname(os.path.abspath(__file__))


class InferWrap:
    def __init__(self, model_dir="model", TPEs=1, core_ids=None, max_inflight=None):
        model_dir = model_dir if os.path.isabs(model_dir) else os.path.join(get_current_dir(), model_dir)
        model_path = self.get_model_path(model_dir)
        if core_ids is None:
            core_ids = [0] if int(TPEs) <= 1 else [0, 1, 2]
        core_ids = list(core_ids) or [0]
        self.TPEs = min(max(1, int(TPEs)), len(core_ids))
        self.rknn_pool = rknnPoolExecutor(
            rknnModel=model_path,
            TPEs=self.TPEs,
            func=myFunc,
            core_ids=core_ids,
            max_inflight=max_inflight or self.TPEs,
        )

    def get_model_path(self, model_dir):
        preferred = os.path.join(model_dir, "rknn_lt.rknn")
        if os.path.exists(preferred):
            return preferred
        files = sorted(
            f for f in glob.glob(os.path.join(model_dir, "*.rknn"))
            if "seg" not in os.path.basename(f).lower() and "pp" not in os.path.basename(f).lower()
        )
        if not files:
            raise FileNotFoundError(f"detection model not found in {model_dir}")
        return files[0]

    def infer(self, img):
        return self.rknn_pool.infer(img)

    def infer_current(self, img, frame_id=None, timestamp=None):
        meta = {
            "frame_id": frame_id,
            "timestamp": timestamp,
        }
        return self.rknn_pool.infer_current(img, meta=meta)

    def __call__(self, *args, **kwargs):
        return self.infer(*args, **kwargs)

    def release(self):
        self.rknn_pool.release()

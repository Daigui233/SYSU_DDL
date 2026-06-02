import glob
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

try:
    from .rknnpool import rknnPoolExecutor
    from .seg_func import myFunc
except ImportError:
    from rknnpool import rknnPoolExecutor
    from seg_func import myFunc


def get_current_dir():
    return os.path.dirname(os.path.abspath(__file__))


class PPSegInfer:
    def __init__(self, model_dir="model", TPEs=1, core_ids=None, max_inflight=None):
        model_dir = model_dir if os.path.isabs(model_dir) else os.path.join(get_current_dir(), model_dir)
        model_path = self.get_model_path(model_dir)
        core_ids = list(core_ids or [1]) or [1]
        self.TPEs = min(max(1, int(TPEs)), len(core_ids))
        self.rknn_pool = rknnPoolExecutor(
            rknnModel=model_path,
            TPEs=self.TPEs,
            func=myFunc,
            core_ids=core_ids,
            max_inflight=max_inflight or 1,
        )

    def get_model_path(self, model_dir):
        patterns = ("pp_seg*.rknn", "ppseg*.rknn")
        files = []
        for pattern in patterns:
            files.extend(sorted(glob.glob(os.path.join(model_dir, pattern))))
        if not files:
            rules = ", ".join(os.path.join(model_dir, p) for p in patterns)
            raise FileNotFoundError(f"segmentation model not found, rules: {rules}")
        return files[0]

    def infer(self, img):
        return self.rknn_pool.infer(img)

    def __call__(self, *args, **kwargs):
        return self.infer(*args, **kwargs)

    def release(self):
        self.rknn_pool.release()

from collections import deque
from concurrent.futures import ThreadPoolExecutor
import sys

import cloudpickle
from rknnlite.api import RKNNLite


CORE_MASKS = {
    0: RKNNLite.NPU_CORE_0,
    1: RKNNLite.NPU_CORE_1,
    2: RKNNLite.NPU_CORE_2,
    -1: RKNNLite.NPU_CORE_0_1_2,
}


def initRKNN(rknnModel="model.rknn", core_id=0):
    rknn_lite = RKNNLite()
    ret = rknn_lite.load_rknn(rknnModel)
    if ret != 0:
        raise RuntimeError(f"load RKNN failed: {rknnModel}, ret={ret}")

    core_mask = CORE_MASKS.get(core_id)
    ret = rknn_lite.init_runtime(core_mask=core_mask) if core_mask is not None else rknn_lite.init_runtime()
    if ret != 0:
        raise RuntimeError(f"init RKNN runtime failed: {rknnModel}, core={core_id}, ret={ret}")

    print(f"{rknnModel}\tdone(core={core_id})")
    return rknn_lite


def initRKNNs(rknnModel="model.rknn", TPEs=1, core_ids=None):
    sys.modules["pickle"] = cloudpickle
    core_ids = list(core_ids or [0]) or [0]
    return [initRKNN(rknnModel, core_ids[i % len(core_ids)]) for i in range(max(1, int(TPEs)))]


class rknnPoolExecutor:
    def __init__(self, rknnModel, TPEs, func, core_ids=None, max_inflight=None):
        self.TPEs = max(1, int(TPEs))
        self.max_inflight = max(1, min(int(max_inflight or self.TPEs), self.TPEs))
        self.rknnPool = initRKNNs(rknnModel, self.TPEs, core_ids=core_ids)
        self.pool = ThreadPoolExecutor(max_workers=self.TPEs)
        self.func = func
        self.futures = deque()
        self.submit_index = 0

    def _submit(self, frame):
        if len(self.futures) >= self.max_inflight:
            return False
        rknn = self.rknnPool[self.submit_index % self.TPEs]
        self.futures.append(self.pool.submit(self.func, rknn, frame.copy()))
        self.submit_index += 1
        return True

    def put(self, frame):
        return self._submit(frame)

    def get(self, block=False):
        if not self.futures:
            return None, False
        idx = None
        if block:
            idx = 0
        else:
            for i, fut in enumerate(self.futures):
                if fut.done():
                    idx = i
                    break
            if idx is None:
                return None, False
        fut = self.futures[idx]
        del self.futures[idx]
        try:
            return fut.result(), True
        except Exception as exc:
            print(f"RKNN worker failed: {exc}")
            return None, False

    def infer(self, frame):
        result, flag = self.get(block=False)
        self._submit(frame)
        return result, flag

    def infer_current(self, frame, meta=None):
        meta = dict(meta or {})
        rknn = self.rknnPool[self.submit_index % self.TPEs]
        self.submit_index += 1
        try:
            return self.func(rknn, frame.copy()), True, meta
        except Exception as exc:
            print(f"RKNN current inference failed: {exc}")
            return None, False, meta

    def release(self):
        self.pool.shutdown(wait=True)
        for rknn_lite in self.rknnPool:
            try:
                rknn_lite.release()
            except Exception:
                pass

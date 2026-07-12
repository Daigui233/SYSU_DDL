from queue import Queue
from rknnlite.api import RKNNLite
from concurrent.futures import ThreadPoolExecutor, as_completed, ProcessPoolExecutor
import cloudpickle
import sys


def initRKNN(rknnModel="yolov.rknn", id=0):
    rknn_lite = RKNNLite()
    ret = rknn_lite.load_rknn(rknnModel)
    if ret != 0:
        rknn_lite.release()
        raise RuntimeError(f"load RKNN model failed: {rknnModel}, ret={ret}")
    if id == 0:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
    elif id == 1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
    elif id == 2:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
    elif id == -1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    else:
        ret = rknn_lite.init_runtime()
    if ret != 0:
        rknn_lite.release()
        raise RuntimeError(f"init RKNN runtime failed: {rknnModel}, core={id}, ret={ret}")
    print(rknnModel, "\t\tdone")
    return rknn_lite


def initRKNNs(rknnModel="yolo.rknn", TPEs=1, use_all_cores=False):
    sys.modules['pickle'] = cloudpickle
    rknn_list = []
    try:
        for i in range(TPEs):
            core_id = -1 if use_all_cores and TPEs == 1 else i % 3
            rknn_list.append(initRKNN(rknnModel, core_id))
    except Exception:
        for rknn_lite in rknn_list:
            rknn_lite.release()
        raise
    return rknn_list


class rknnPoolExecutor():
    def __init__(self, rknnModel, TPEs, func, use_all_cores=False):
        self.TPEs = TPEs
        self.queue = Queue()
        self.rknnPool = initRKNNs(rknnModel, TPEs, use_all_cores)
        self.pool = ThreadPoolExecutor(max_workers=TPEs) if TPEs > 1 else None
        # self.pool = ProcessPoolExecutor(max_workers=TPEs)
        self.func = func
        self.num = 0
        self.released = False

    def put(self, frame):
        if self.pool is None:
            self.queue.put(self.func(self.rknnPool[0], frame))
            return
        self.queue.put(self.pool.submit(
            self.func, self.rknnPool[self.num % self.TPEs], frame))
        self.num += 1

    def get(self):
        if self.queue.empty():
            return None, False
        item = self.queue.get()
        return (item.result() if self.pool is not None else item), True

    def run(self, frame):
        if self.released:
            raise RuntimeError("RKNN runtime has been released")
        if self.TPEs != 1:
            raise RuntimeError("synchronous run requires exactly one RKNN runtime")
        return self.func(self.rknnPool[0], frame)

    def release(self):
        if self.released:
            return
        self.released = True
        if self.pool is not None:
            self.pool.shutdown()
        for rknn_lite in self.rknnPool:
            rknn_lite.release()

import cv2, sys, os, glob
import numpy as np
import platform
# from synset_label import labels
from rknnlite.api import RKNNLite
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# from camera import Camera
from rknnpool import rknnPoolExecutor
from func import myFunc

def get_current_dir():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return current_dir


class InferWrap():
    def __init__(self, model_dir="model", TPEs=1):
        model_dir = os.path.join(get_current_dir(), model_dir)
        model_path = self.get_model_path(model_dir)
        self.TPEs = TPEs
        self.rknn_pool = rknnPoolExecutor(
            rknnModel=model_path,
            TPEs=self.TPEs,
            func=myFunc)
        self.pool_flag = False

    def pool_init(self, img):
        # 初始化线程池加载数据
        for i in range(self.TPEs + 1):
            self.rknn_pool.put(img)

    def get_model_path(self, model_dir):
        model_path  = glob.glob(model_dir + "/*.rknn")[0]
        # cfg_path = glob.glob(model_dir + "/*.names")[0]
        # return model_path, params_path
        return model_path

    def infer(self, img):
        if not self.pool_flag:
            self.pool_init(img)
            self.pool_flag = True
        self.rknn_pool.put(img)
        return self.rknn_pool.get()
    
    def __call__(self, *args, **kwds):
        return self.infer(*args, **kwds)
    
    def release(self):
        self.rknn_pool.release()

if __name__ == '__main__':


    # cap = Camera(0)
    infer = InferWrap("model", TPEs=1)

    ori_img = cv2.imread('./bus.jpg')
    

    import time
    # time_ls = time.time()
    # frame
    frames, loopTime, initTime = 0, time.time(), time.time()
    cnt_end = 1000
    result, flag = infer.infer(ori_img)
    cv2.imwrite("result.jpg", result)
    infer.release()

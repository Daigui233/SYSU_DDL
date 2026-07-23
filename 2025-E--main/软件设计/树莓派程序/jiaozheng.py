    #使用方法：在其他文件里导入并创建矫正器（确保标定文件与摄像头分辨率匹配）
#from jiaozheng import CameraCalibration
#calibrator = CameraCalibration("calibration_result.yaml")
    #然后对捕获的frame处理：
#frame = calibrator.undistort(frame)




import cv2
import numpy as np
import yaml

class CameraCalibration:
    def __init__(self, calibration_file="calibration_result.yaml", balance=0.0):
        """初始化摄像头标定和畸变矫正参数"""
        # 加载标定参数
        self.DIM, self.K, self.D = self._load_calibration(calibration_file)
        
        # 计算矫正映射
        self.balance = balance
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, self.DIM, np.eye(3), balance=self.balance
        )
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.new_K, self.DIM, cv2.CV_16SC2
        )
        
    def _load_calibration(self, file_path):
        """从YAML文件加载标定参数"""
        try:
            with open(file_path, "r") as f:
                data = yaml.safe_load(f)
            DIM = tuple(data["DIM"])
            K = np.array(data["K"], dtype=np.float64)
            D = np.array(data["D"], dtype=np.float64).reshape(-1, 1)
            return DIM, K, D
        except FileNotFoundError:
            raise FileNotFoundError(f"标定文件 {file_path} 未找到！")
        except Exception as e:
            raise RuntimeError(f"加载标定参数失败：{e}")
    
    def undistort(self, frame):
        """对输入的图像帧进行畸变矫正"""
        return cv2.remap(
            frame, self.map1, self.map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )
    
    def get_camera_params(self):
        """获取相机内参和畸变系数"""
        return self.K, self.D, self.new_K
    
    def get_dimensions(self):
        """获取图像尺寸"""
        return self.DIM


# ---------------------- 示例用法（直接运行此文件时） ----------------------
if __name__ == "__main__":
    # 创建矫正器实例
    try:
        calibrator = CameraCalibration()
    except Exception as e:
        print(f"初始化失败: {e}")
        exit()
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    
    # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, calibrator.get_dimensions()[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calibrator.get_dimensions()[1])
    
    print("按 'q' 键退出程序")
    
    # 实时矫正循环
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取画面")
            break
        
        # 矫正图像
        undistorted = calibrator.undistort(frame)
        
        # 显示结果
        cv2.imshow("Original", frame)
        cv2.imshow("Undistorted", undistorted)
        
        # 按键退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
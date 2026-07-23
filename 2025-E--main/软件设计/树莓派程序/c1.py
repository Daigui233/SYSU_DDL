import cv2
import numpy as np
import serial
import time
from jiaozheng import CameraCalibration

# 相机校准
calibrator = CameraCalibration(r"/home/pi/Desktop/last/calibration_result.yaml")

# A4纸实际参数（厘米）
A4_WIDTH_CM = 17.1   # 短边
A4_HEIGHT_CM = 26.0  # 长边
A4_ASPECT_RATIO = A4_HEIGHT_CM / A4_WIDTH_CM  # 长宽比
ASPECT_RATIO_TOLERANCE = 0.4# 比例容忍度

# 串口配置
SERIAL_PORT = '/dev/ttyUSB0'  # Linux端口，Windows为'COM3'等
BAUD_RATE = 115200
TIMEOUT = 1


class A4RectangleDetector:
    def __init__(self):
        self.min_area = 2000  # 最小面积阈值
        self.max_area = 80000  # 最大面积阈值
        self.contour_epsilon = 0.1  # 轮廓近似系数
        self.angle_threshold = 70  # 矩形角阈值（接近直角）
        self.last_center = None  # 上一次原图中心点
        self.pixels_per_cm = None  # 像素/厘米比例
        self.detected_area = 0  # 检测到的面积
        self.warped = None  # 透视变换后的正视图
        self.warped_center = None  # 正视图中的中心
        # 正视图尺寸（与A4比例一致）
        self.target_width = 171  # 对应短边
        self.target_height = 260  # 对应长边

    def _order_points(self, pts):
        """顺时针排序角点（左上->右上->右下->左下）"""
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        top_left = pts[np.argmin(s)]
        bottom_right = pts[np.argmax(s)]
        top_right = pts[np.argmax(diff)]
        bottom_left = pts[np.argmin(diff)]
        return np.array([top_left, top_right, bottom_right, bottom_left], dtype="float32")

    def _is_rectangle(self, approx):
        """判断是否为矩形（检查直角）"""
        if len(approx) != 4:
            return False
            
        corners = approx.reshape(4, 2).astype(np.float32)
        vectors = []
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i+1)%4]
            vectors.append(p2 - p1)
            
        # 检查每个角是否为直角
        for i in range(4):
            v1 = vectors[i]
            v2 = vectors[(i+1)%4]
            dot = np.dot(v1, v2)
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            
            if norm1 == 0 or norm2 == 0:
                return False
                
            cos_theta = abs(dot / (norm1 * norm2))
            cos_theta = np.clip(cos_theta, 0.0, 1.0)
            angle = np.arccos(cos_theta) * 180 / np.pi
            
            if angle < self.angle_threshold:
                return False
        return True

    def _check_aspect_ratio(self, corners):
        """检查矩形长宽比是否接近A4纸"""
        # 计算相邻两边的长度（矩形的宽和长）
        side1 = np.linalg.norm(corners[0] - corners[1])  # 边1长度
        side2 = np.linalg.norm(corners[1] - corners[2])  # 边2长度
        
        # 计算长宽比（长边/短边）
        if side1 > side2:
            aspect_ratio = side1 / side2
        else:
            aspect_ratio = side2 / side1
            
        # 检查是否在A4比例的容忍范围内
        min_ratio = A4_ASPECT_RATIO * (1 - ASPECT_RATIO_TOLERANCE)
        max_ratio = A4_ASPECT_RATIO * (1 + ASPECT_RATIO_TOLERANCE)
        return min_ratio <= aspect_ratio <= max_ratio

    def _calculate_pixel_ratio(self, corners):
        """计算像素/厘米比例"""
        p1, p2, p3, p4 = corners
        width_pixels = np.linalg.norm(p1 - p2)  # 短边像素长度
        height_pixels = np.linalg.norm(p2 - p3)  # 长边像素长度
        
        ratio_width = width_pixels / A4_WIDTH_CM
        ratio_height = height_pixels / A4_HEIGHT_CM
        return (ratio_width + ratio_height) / 2  # 平均比例

    def _perspective_transform(self, corners):
        """计算透视变换矩阵并生成正视图"""
        # 目标正视图的四个角点（左上、右上、右下、左下）
        target_corners = np.array([
            [0, 0],
            [self.target_width - 1, 0],
            [self.target_width - 1, self.target_height - 1],
            [0, self.target_height - 1]
        ], dtype="float32")
        # 计算透视变换矩阵
        M = cv2.getPerspectiveTransform(corners, target_corners)
        # 生成正视图（使用原图彩色图，便于观察）
        self.warped = cv2.warpPerspective(
            self.original_frame,  # 用彩色原图生成正视图
            M, 
            (self.target_width, self.target_height)
        )
        # 计算逆变换矩阵（用于将正视图中心映射回原图）
        self.M_inv = cv2.getPerspectiveTransform(target_corners, corners)
        # 正视图中心（物理中心）
        self.warped_center = (self.target_width // 2, self.target_height // 2)
        return self.warped_center

    def _get_original_center(self):
        """将正视图中心映射回原图，得到原图中的真实中心"""
        if self.warped_center is None or not hasattr(self, 'M_inv'):
            return None
        # 正视图中心坐标
        warped_center_np = np.array([[self.warped_center]], dtype="float32")
        # 映射回原图
        original_center = cv2.perspectiveTransform(warped_center_np.reshape(-1, 1, 2), self.M_inv)
        return (int(original_center[0][0][0]), int(original_center[0][0][1]))

    def detect_and_draw(self, frame):
        """检测符合A4比例的矩形，计算中心并绘制"""
        self.original_frame = frame.copy()  # 保存彩色原图用于透视变换
        self.original_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.bilateralFilter(self.original_gray, 9, 75, 75)
        edges = cv2.Canny(blur, 50, 150)
        
        # 获取所有轮廓及层级关系
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        result = frame.copy()
        min_area = self.max_area
        min_rect_corners = None
        self.detected_area = 0
        self.warped = None  # 重置正视图
        self.warped_center = None  # 重置正视图中心

        # 筛选符合条件的矩形（支持单轮廓或内轮廓）
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
                
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.contour_epsilon * perimeter, True)
            
            # 检查是否为矩形
            if not self._is_rectangle(approx):
                continue
                
            # 检查长宽比
            corners = approx.reshape(4, 2).astype(np.float32)
            if not self._check_aspect_ratio(corners):
                continue
                
            # 角点排序
            ordered_corners = self._order_points(corners)
                
            # 优先选择面积较小的（更可能是内矩形）
            if area < min_area:
                min_area = area
                min_rect_corners = ordered_corners
                self.detected_area = area

        # 处理检测到的矩形
        original_center = None  # 原图中的矩形中心
        if min_rect_corners is not None:
            # 透视变换：生成正视图并计算中心
            self._perspective_transform(min_rect_corners)
            # 将正视图中心映射回原图
            original_center = self._get_original_center()
            self.last_center = original_center  # 保存原图中心
            
            self.pixels_per_cm = self._calculate_pixel_ratio(min_rect_corners)
            
            # 在原图上绘制检测到的矩形
            cv2.polylines(result, [min_rect_corners.astype(np.int32)], True, (0, 0, 255), 2)

        else:
            cv2.putText(result, "No inner rectangle found", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.last_center = None
            self.pixels_per_cm = None

        # 在原图上绘制矩形中心点（蓝色小圆）
        if original_center is not None:
            cv2.circle(result, original_center, 4, (255, 0, 0), -1)  # 实心小圆

        return result, original_center  # 返回原图和原图中的中心坐标


def main():
    detector = A4RectangleDetector()
    cap = cv2.VideoCapture(0)
    cap.set(6, cv2.VideoWriter.fourcc('M','J','P','G'))
    cap.set(5, 60)  # 保持60帧设置
    # 初始化串口
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2)
        print(f"serial_connected: {SERIAL_PORT}")
    except Exception as e:
        print(f"serial_init_error: {e}")
    
    # 帧数计算
    prev_time = 0
    fps = 0
    
    try:
        while True:
            current_time = cv2.getTickCount()
            time_elapsed = (current_time - prev_time) / cv2.getTickFrequency()
            if time_elapsed > 0:
                fps = 0.9 * fps + 0.1 * (1 / time_elapsed)
            prev_time = current_time
            
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, -1)
            frame = calibrator.undistort(frame)  # 如需畸变校正可启用
            output, rect_center = detector.detect_and_draw(frame)
            
            # 画面中心点（激光中心，黄色小圆）
            height, width = output.shape[:2]
            #laser_center = (width // 2, height // 2)
            if detector.detected_area == 0:
                laser_center = (330,251)
            else:
                width_center = int((int(detector.detected_area) * int(detector.detected_area) *(-1.928e-09))+int(detector.detected_area)*0.0004398 + 318.7)
                laser_center = (width_center,251)
            cv2.circle(output, laser_center, 2, (0, 255, 255), -1)  # 激光中心小圆
            cv2.putText(output, f"laser_center: ({laser_center[0]}, {laser_center[1]})", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # 内矩形中心文本和偏移量
            if rect_center is not None:
                cv2.putText(output, f"Inner Rect Center: ({rect_center[0]}, {rect_center[1]})", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                offset_x = rect_center[0] - laser_center[0]  
                offset_y = rect_center[1] - laser_center[1]
                cv2.putText(output, f"Offset: ({offset_x}, {offset_y})", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            
            # 帧率显示
            fps_text = f"FPS: {int(fps)}"
            cv2.putText(output, fps_text, 
                       (width - 120, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 面积显示（右下角）
            if detector.detected_area > 0:
                area_text = f"Area: {int(detector.detected_area)} px"
                cv2.putText(output, area_text,
                           (width - 200, height - 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # 独立窗口显示透视变换图
            if detector.warped is not None:
                # 在正视图上绘制中心（红色小圆）
                warped_with_center = detector.warped.copy()
                cv2.circle(warped_with_center, detector.warped_center, 5, (0, 0, 255), -1)
                cv2.putText(warped_with_center, f"Warped Center: {detector.warped_center}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Warped View", warped_with_center)
            
            # 串口发送
            if ser and ser.is_open and rect_center is not None:
                data = f"@[{rect_center[0]},{rect_center[1]}];[{laser_center[0]},{laser_center[1]}]%\r\n"
                try:
                    ser.write(data.encode())
                    cv2.putText(output, "Sending inner rect offset...", (10, height - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                except Exception as e:
                    print(f"send_error: {e}")
            
            # 显示主窗口
            cv2.imshow("Inner Rectangle Detector", output)
            
            # 退出条件
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()  # 关闭所有窗口
        if ser and ser.is_open:
            ser.close()
            print("serial_closed")

if __name__ == "__main__":
    main()

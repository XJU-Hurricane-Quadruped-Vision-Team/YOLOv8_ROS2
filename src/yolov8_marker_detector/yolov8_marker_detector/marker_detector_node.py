import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import sys
import numpy as np

class MarkerDetectorNode(Node):
    def __init__(self):
        super().__init__('marker_detector_node')

        self.publisher_ = self.create_publisher(String, 'obstacle_info', 10)

        pkg_dir = get_package_share_directory('yolov8_marker_detector')
        model_path = os.path.join(pkg_dir, 'models', 'train_best0421.pt')
        self.get_logger().info(f"加载模型: {model_path}")
        self.model = YOLO(model_path)
        self.model.to('cpu')

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("摄像头打开失败！")
            return

        window_name = "YOLOv8 检测"
        cv2.namedWindow(window_name)
        cv2.imshow(window_name, np.zeros((480, 640, 3), dtype=np.uint8))
        self.get_logger().info(f"窗口 '{window_name}' 已创建")
        
        self.trackbar_enabled = False
        try:
            cv2.createTrackbar("Conf %", window_name, 30, 100, lambda x: None)
            cv2.createTrackbar("IOU %", window_name, 45, 100, lambda x: None)
            self.trackbar_enabled = True
            self.get_logger().info(f"滑动条已添加到窗口 '{window_name}'")
        except Exception as e:
            self.get_logger().warn(f"滑动条创建失败: {e}")

        self.timer = self.create_timer(0.0, self.detect_from_camera)

    def detect_from_camera(self):
        if cv2.getWindowProperty("YOLOv8 检测", cv2.WND_PROP_VISIBLE) < 1:
            self.get_logger().info("检测窗口已关闭，退出节点")
            self.destroy_node()
            rclpy.shutdown()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("无法读取摄像头图像")
            return
        
        conf = 0.3
        iou  = 0.45

        if self.trackbar_enabled:
            try:
                conf = cv2.getTrackbarPos("Conf %", "YOLOv8 检测") / 100.0
                iou  = cv2.getTrackbarPos("IOU %", "YOLOv8 检测") / 100.0
                if not (0.0 <= conf <= 1.0):
                    conf = 0.3
                if not (0.0 <= iou <= 1.0):
                    iou = 0.45
            except Exception as e:
                self.get_logger().warn(f"滑动条创建失败: {e}")

        try:
            print((conf, iou))
            results = self.model.predict(
                source=frame,
                device='cpu',  
                imgsz=640,
                conf=conf,
                iou=iou,
                half=False,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"模型预测出错：{e}")
            return

        detections = []
        annotated = frame.copy()
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf_v = float(box.conf[0])
            label = self.model.names[cls_id]
            detections.append(f"{label}({conf_v:.2f})")
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{label} {conf_v:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        msg = String()
        msg.data = ', '.join(detections) if detections else "未检测到标识物"
        self.publisher_.publish(msg)
        self.get_logger().info(f"检测结果: {msg.data}")

        cv2.imshow("YOLOv8 检测", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("按键 'q' 检测到，退出节点")
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("检测窗口已关闭，退出节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

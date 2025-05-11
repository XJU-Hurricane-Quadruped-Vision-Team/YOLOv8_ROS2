from ultralytics import YOLO
import cv2

class MarkerDetectorNode:
    def __init__(self):
        # 加载模型
        model_path = "D:\\code\\python\\yolov8_train\\train_best.pt"
        print(f"加载模型: {model_path}")
        self.model = YOLO(model_path)
        self.model.to('cpu')

        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("摄像头打开失败！")
            return

        # 创建窗口和滑动条
        window_name = "YOLOv8 检测"
        cv2.namedWindow(window_name)
        print(f"窗口 '{window_name}' 已创建")

        try:
            cv2.createTrackbar("Conf %", window_name, 30, 100, lambda x: None)
            cv2.createTrackbar("IOU %", window_name, 45, 100, lambda x: None)
            print(f"滑动条已添加到窗口 '{window_name}'")
        except Exception as e:
            print(f"滑动条创建失败: {e}")

    def detect_from_camera(self):
        while True:
            # 检查窗口是否关闭
            if cv2.getWindowProperty("YOLOv8 检测", cv2.WND_PROP_VISIBLE) < 1:
                print("检测窗口已关闭，退出程序")
                break

            # 读取摄像头帧
            ret, frame = self.cap.read()
            if not ret:
                print("无法读取摄像头图像")
                break

            # 获取滑动条值
            conf = cv2.getTrackbarPos("Conf %", "YOLOv8 检测") / 100.0
            iou = cv2.getTrackbarPos("IOU %", "YOLOv8 检测") / 100.0

            try:
                # 使用 YOLO 模型进行预测
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
                print(f"模型预测出错：{e}")
                break

            # 处理检测结果
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

            # 显示检测结果
            detection_msg = ', '.join(detections) if detections else "未检测到标识物"
            print(f"检测结果: {detection_msg}")
            cv2.imshow("YOLOv8 检测", annotated)

            # 按键退出
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("按键 'q' 检测到，退出程序")
                break

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()

# 主函数
if __name__ == '__main__':
    node = MarkerDetectorNode()
    node.detect_from_camera()
    node.destroy_node()
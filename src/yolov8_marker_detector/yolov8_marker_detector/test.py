import cv2
import numpy as np

# 创建一个空白图像
image = 255 * np.ones((480, 640, 3), dtype=np.uint8)

# 创建窗口
cv2.namedWindow("Test Window")

# 在窗口中显示图像
cv2.imshow("Test Window", image)

# 等待用户按键
cv2.waitKey(0)

# 销毁窗口
cv2.destroyAllWindows()
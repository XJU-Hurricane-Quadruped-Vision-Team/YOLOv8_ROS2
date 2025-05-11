1. 挂载 Google Drive

```
from google.colab import drive
drive.mount('/content/drive')
```

2. 检查 GPU 是否可用

```
# 查看显卡驱动
!/opt/bin/nvidia-smi
!nvidia-smi

import torch
print('CUDA available:', torch.cuda.is_available(), 'GPU count:', torch.cuda.device_count())
```

3. 解压项目

```
!unzip /content/drive/MyDrive/Colab Notebooks/yolov8_train.zip -d /content/drive/MyDrive/Colab Notebooks/yolov8_train
```

4. 进入项目目录

```
%cd /content/yolov8_train
```

5. 安装依赖

```
!pip uninstall -y ultralytics
!pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117
!pip install ultralytics
!pip install -r requirements.txt --no-deps

import torch
print('CUDA:', torch.cuda.is_available(), 'GPUs:', torch.cuda.device_count())
```

6. 启动 TensorBoard

```
%load_ext tensorboard
%tensorboard --logdir runs/train
```

7. 训练并生成 `best.pt`

```
!yolo train data=ultralytics/yolo/data/datasets/mydatas.yaml model=yolov8n.pt epochs=300 imgsz=640 batch=8 workers=2 device=0 name=colab_train
```

8. 拷贝并下载 `best.pt`

```
路径：ultralytics/yolo/v8/detect/runs/detect
!ls runs/detect/colab_train/weights
```


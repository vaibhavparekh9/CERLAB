from ultralytics import YOLO

model = YOLO("yolov8n.pt")

results = model.train(data="/home/vsparekh/isaac_py_tuts/src/camera/detect/geo shapes detection.v1i.yolov11/data.yaml", 
                      epochs=100, imgsz=640, batch=6, device='cuda')

# # (Optional) Evaluate on your validation set
# metrics = model.val(data="data.yaml")
# print(metrics)pip install ultralytics

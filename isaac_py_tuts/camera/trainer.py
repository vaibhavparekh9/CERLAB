from ultralytics import YOLO

model = YOLO("yolov8n.pt")

results = model.train(data="/home/vsparekh/isaac_py_tuts/src/camera/detect/geo shapes detection.v1i.yolov11/data.yaml", 
                      epochs=100, imgsz=640, batch=6, device='cuda')

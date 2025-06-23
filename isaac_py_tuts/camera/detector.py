from ultralytics import YOLO
import cv2

model = YOLO(r'/home/vsparekh/Outlet detection/Shape detector/runs/detect/train2/weights/best.pt')

img_path = r'/home/vsparekh/Outlet detection/Shape detector/capture.png'

results = model.predict(img_path, conf=0.25, iou=0.45, device='cuda:0')

annotated = results[0].plot()

# # (Optional) resize for display
# h, w = annotated.shape[:2]
# scale = 0.5
# annotated = cv2.resize(annotated, (int(w*scale), int(h*scale)))

cv2.imshow('Shape Detections', annotated)
cv2.waitKey(15000)
cv2.destroyAllWindows()
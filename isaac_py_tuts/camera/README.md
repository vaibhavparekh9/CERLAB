This tutorial is on how to:

1. Spawn World and few basic objects (like Cube/Sphere/Cone)
2. Spawn a camera and capture an image using the same
3. Running a YOLO model to detect shape of the object

Note: dataset taken from Roboflow (https://universe.roboflow.com/semih-4gisc/geo-shapes-detection/dataset/1/)

Files: 
camera.py: spawn world, objects, and camera; and capture and save an image
trainer.py: train the YOLO model
detector.py: takes the captured image; and gives inference (bounding box)
best.pt: weights

from ultralytics import YOLO

model = YOLO('/root/ida_ws/vision/models/best.pt')
model.export(format='engine', imgsz=640, half=True, device=0)
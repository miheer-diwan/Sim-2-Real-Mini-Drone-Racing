from ultralytics import YOLO

# Load a model
model = YOLO('/home/oliver/RBE595_Drone/Project_2/YourDirectoryID_p2a/src/yolov8n-seg.pt')  # load a pretrained model (recommended for training)

# Train the model
results = model.train(data='/home/oliver/GateDataset/yolo_train/GateCornerSeg.v1i.yolov8/data.yaml',device=0,resume=False, epochs=150, imgsz=720, save=True, batch=-1, workers = 16)
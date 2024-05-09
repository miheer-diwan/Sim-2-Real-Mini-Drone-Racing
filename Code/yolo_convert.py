from ultralytics import YOLO

combined_model = YOLO('/home/oliver/RBE595_Drone/Project_2/YourDirectoryID_p2a/src/runs/segment/train7/weights/best.pt')

combined_model.export(format = "onnx")
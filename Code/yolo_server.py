from ultralytics import YOLO
import cv2
import numpy as np
import random
import math
import json
import zmq
import time
import threading

class FrameReceiver(threading.Thread):
    def __init__(self, ip="127.0.0.1",image_width = 960, image_height = 720, image_channels = 3, port=5560, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.image_width = image_width
        self.image_height = image_height
        self.image_channels = image_channels
        sub_url = "tcp://{}:{}".format(ip, port)
        self.ctx = zmq.Context()
        self.image_sub = self.ctx.socket(zmq.SUB)
        self.image_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.image_sub.setsockopt(zmq.RCVHWM, 1)
        self.image_sub.bind(sub_url)
        self.most_recent_frame = None
        self.lock = threading.Lock()
        self.last_frame_time = time.time() - 10.0
        time.sleep(0.5)
        
        print("Frame receiver subscriber connected to: {}\n".format(sub_url))

    def run(self):
        while True:
            try:
                image_buffer = self.image_sub.recv(flags=zmq.NOBLOCK)
                # print("Frame received")
                image = np.frombuffer(image_buffer, dtype=np.uint8)
                image = image.reshape((self.image_height, self.image_width, self.image_channels))
                with self.lock:
                    self.most_recent_frame = image.copy()
                    self.last_frame_time = time.time()
            except zmq.Again as e:
                pass
            
            if (time.time() - self.last_frame_time > 0.1) and (self.most_recent_frame is not None):
                with self.lock:
                    self.most_recent_frame = None
                    cv2.destroyAllWindows()
            time.sleep(0.0001)

    def getMostRecentFrame(self):
        most_recent_frame = None
        with self.lock:
            if self.most_recent_frame is not None:
                most_recent_frame = self.most_recent_frame.copy()
                self.most_recent_frame = None
                # print("getMostRecentFrame", most_recent_frame.shape)

        return most_recent_frame
            

class GateDetect:
    def __init__(self, model_path):
        self.gate_detection_model = YOLO(model_path)

        self.gate_color = (255, 0, 0, 255)
        self.corner_color = (0, 255, 0, 255)
        color_1 = (255, 0, 0, 255)
        color_2 = (0, 255, 0, 255)
        color_3 = (0, 0, 255, 255)
        color_4 = (255, 0, 255, 255)
        color_5 = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), 255)
        self.colors = [color_1, color_2, color_3, color_4, color_5]

    def group_gates_and_corners(self, gates, corners, center_dist_vs_gate_radius_ratio_range = [0.8 ,1.05], radius_ratio_min = 0.5):
        gates.sort(key=lambda a: a[1], reverse=True)
        corners.sort(key=lambda a: a[1], reverse=True)

        grouped_corners = []
        for gate in gates:
            grouped_corners.append([])
            if gate[1] > 0:
                count = 0
                idx = 0
                while idx < len(corners):
                    center_dist = math.dist(corners[idx][0], gate[0])
                    center_dist_vs_gate_radius_ratio = center_dist/gate[1]
                    radius_ratio = corners[idx][1] / gate[1]
                    if center_dist_vs_gate_radius_ratio > center_dist_vs_gate_radius_ratio_range[0] and \
                        center_dist_vs_gate_radius_ratio <center_dist_vs_gate_radius_ratio_range[1] and \
                        radius_ratio > radius_ratio_min:
                        grouped_corners[-1].append(corners[idx])
                        corners.pop(idx)
                        idx -=1
                        count += 1
                    idx += 1

                    if count >=4:
                        continue
                
        return grouped_corners
    
    def detect_gate(self, image, conf = 0.8):
        raw_result = self.gate_detection_model.predict(image, stream=False, imgsz=640, conf=0.8)
        gate_and_corners = self.process_model_result(raw_result)
        return gate_and_corners
    
    def process_model_result(self, model_result, image = None):
        for r in model_result:
            boxes = r.boxes  # Boxes object for bbox outputs
            obj_type = boxes.cls.cpu().numpy().astype(int)
            masks = r.masks  # Masks object for segment masks outputs
            # probs = r.probs  # Class probabilities for classification outputs
        
        detected_gates = {}
        detected_gates["gates"] = []
        if masks != None:
            gates = []
            corners = []
            for idx,contour in enumerate(masks.xy):
                (x,y),radius = cv2.minEnclosingCircle(contour)
                center = (int(x),int(y))
                radius = int(radius)
                if obj_type[idx] == 0:
                    gates.append((center,radius))
                    if image is not None: cv2.circle(image,center,radius,self.gate_color,1)
                    if image is not None: cv2.circle(image,center,2,self.gate_color,1)
                else:
                    corners.append((center,radius))
                    if image is not None: cv2.circle(image,center,radius,self.corner_color,1)
                    if image is not None: cv2.circle(image,center,2,self.corner_color,1)
            
            
            if len(gates) and len(corners):
                grouped_corner = self.group_gates_and_corners(gates,corners,[0.4 , 1.03], 0.25)
                
                for idx, corners in enumerate(grouped_corner):
                    gate = {}
                    gate["gate_center"] =  gates[idx]
                    gate["gate_corners"] = [[],[],[],[]]
                    gate_center = gates[idx][0]
                    if image is not None: cv2.circle(image,gate_center,8,self.colors[idx],6)
                    for corner in corners:
                        center = corner[0]
                        radius = corner[1]
                        #determinine order of corner coords.
                        corner2center_vec = np.array(list(corner[0])) - np.array(list(gate_center))
                        if(corner2center_vec[0] <= 0 and corner2center_vec[1] <= 0):
                            order_idx = 0 #top left corner
                        elif(corner2center_vec[0] >= 0 and corner2center_vec[1] <= 0):
                            order_idx = 1 #top right corner
                        elif(corner2center_vec[0] >= 0 and corner2center_vec[1] >= 0):
                            order_idx = 2 #bottom right corner
                        else:
                            order_idx = 3 #bottom left corner
                        
                        gate["gate_corners"][order_idx] = corner
                        if image is not None: cv2.circle(image,center,8,self.colors[order_idx],6)
                    
                    detected_gates["gates"].append(gate)
                print(detected_gates)
            if image is not None:
                print("Visualize")
                cv2.imshow('Detecttion', image)
                cv2.waitKey(1)
        return detected_gates

class GateDetectionResultPub():
    def __init__(self, ip="127.0.0.1", port=5561):
        self.ctx = zmq.Context()
        pub_url = "tcp://{}:{}".format(ip, port)
        self.pub_socket = self.ctx.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.SNDHWM, 1)
        self.pub_socket.bind(pub_url)
        time.sleep(0.5)
        
        print("Gate detection result pub connected to: {}\n".format(pub_url))

    def sendGateDetectionOutput(self, gate_detection_output_json):
        json_string = json.dumps(gate_detection_output_json).encode('utf-8')
        print(json_string)
        self.pub_socket.send(json_string)

if __name__=="__main__":
    # donot run main.py if imported as a module
    gate_detector = GateDetect('/home/pear/drones/combined_model/best-seg.engine')

    running = threading.Event()
    running.set()
    frame_receiver = FrameReceiver(args=(running, 1))
    frame_receiver.start()

    result_pub = GateDetectionResultPub()

    time.sleep(0.2)

    while True:
        most_recent_frame = frame_receiver.getMostRecentFrame()
        if most_recent_frame is not None:
            print("Frame Received, Detecting Gate")
            detection_result = gate_detector.detect_gate(most_recent_frame)
            result_pub.sendGateDetectionOutput(detection_result)
        time.sleep(0.0001)

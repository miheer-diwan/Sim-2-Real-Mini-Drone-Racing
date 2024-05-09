import cv2
import zmq
import time
import numpy as np

ip="127.0.0.1"
port = "5562"
sub_url = "tcp://{}:{}".format(ip, port)
ctx = zmq.Context()
image_sub = ctx.socket(zmq.SUB)
image_sub.setsockopt_string(zmq.SUBSCRIBE, "")
image_sub.setsockopt(zmq.RCVHWM, 1)
image_sub.bind(sub_url)
print("frame sub connected to: {}\n".format(sub_url))
time.sleep(0.5)

while True:
    try:
        image_buffer = image_sub.recv(flags=zmq.NOBLOCK)
        print("Frame received")
        image = np.frombuffer(image_buffer, dtype=np.uint8)
        image = image.reshape((360, 480, 3))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        print(image.shape)
        cv2.imshow("Image Stream", image)
        cv2.waitKey(1)
    except zmq.Again as e:
        pass
    time.sleep(0.01)
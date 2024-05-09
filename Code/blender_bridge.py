import zmq
import json
import time
import base64
import cv2
import numpy as np
from gray2color import gray2color

class BlenderBridge:
    def __init__(self, ip="127.0.0.1", port=5550):
        pub_url = "tcp://{}:{}".format(ip, port)
        sub_url = "tcp://{}:{}".format(ip, port+1)
        self.ctx = zmq.Context()
        self.pub_socket = self.ctx.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.SNDHWM, 1)
        self.pub_socket.connect(pub_url)
        self.sub_socket = self.ctx.socket(zmq.SUB)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub_socket.setsockopt(zmq.RCVHWM, 1)
        self.sub_socket.connect(sub_url)
        time.sleep(0.5)
        print("Blender Bridge publisher connected to: {}\n".format(pub_url))
        print("Blender Bridge subscriber connected to: {}\n".format(sub_url))
    
    def getRender(self, drone_states, gate_states,
                  render = False,
                render_depth = False, render_seg = False, render_opt = False,
                reset_animation = False,
                change_camera_setting = False,
                image_width = 640,
                image_height = 480,
                K = [[921.170702, 0.000000, 320],
                    [0.000000, 919.018377, 240],
                    [0.000000, 0.000000, 1.000000]]):
        
        topic = 'RenderReq'.encode('utf-8')
        msg = {}
        msg["render"] = render
        msg["reset_animation"] = reset_animation
        msg["change_camera_setting"] = change_camera_setting
        msg["fps"] = 20

        camera_params = {}
        camera_params["render_size"] = [image_width, image_height]
        camera_params["K"] = K
        msg["camera_params"] = camera_params

        render_selection = {}
        render_selection["render_depth"] = render_depth
        render_selection["render_seg"] = render_seg
        render_selection["render_opt"] = render_opt
        msg["render_selection"] = render_selection

        drones = []
        for drone_state in drone_states:
            drone = {}
            drone["frame_id"] = "sample"
            drone["object_type"] = 1
            drone["position_xyz"] = drone_state[0:3]
            drone["oritentation_wxyz"] = drone_state[3:7]
            drones.append(drone)
        msg["drones"] = drones

        objects = []
        for idx, gate_state in enumerate(gate_states):
            gate = {}
            gate["frame_id"] = "Gate.{}".format(idx+1)
            gate["object_type"] = 2
            gate["position_xyz"] = gate_state[0:3]
            gate["oritentation_wxyz"] = gate_state[3:7]
            objects.append(gate)
        msg["objects"] = objects

        msg_str = json.dumps(msg).encode('utf-8')
        self.pub_socket.send_multipart([topic, msg_str])

        if render == True:
            RGBA_buffer, depth_buffer, seg_buffer, opt_buffer = self.sub_socket.recv_multipart()
            RGBA_img = np.frombuffer(RGBA_buffer, dtype=np.uint8)
            RGBA_img = RGBA_img.reshape((image_height, image_width, 4))

            depth_img = None
            seg_img = None
            opt_img = None
            if depth_buffer != b'':
                depth_img = np.frombuffer(depth_buffer, dtype=np.uint8)
                depth_img = depth_img.reshape((image_height, image_width, 1))
            if seg_buffer != b'':
                seg_img = np.frombuffer(seg_buffer, dtype=np.uint8)
                seg_img = np.frombuffer(seg_buffer, dtype=np.uint8)
                seg_img = seg_img.reshape((image_height, image_width))
                # c_pallet = np.array([[[128, 64, 128],
                #         [244, 35, 232],
                #         [70, 70, 70],
                #         [102, 102, 156],
                #         [190, 153, 153]]], np.uint8) / 255
                # seg_img_color = gray2color(seg_img, use_pallet=None, custom_pallet=c_pallet, backend='np')
            if opt_buffer != b'':
                opt_img = np.frombuffer(opt_buffer, dtype=np.uint8)

            return RGBA_img, depth_img, seg_img
        
        return None
        # print(npimg.shape)
        # cv_img = cv2.cvtColor(npimg, cv2.COLOR_BGRA2RGBA)
        # cv2.imshow("RGBA", RGBA_img)
        # if depth_img is not None:
        #     cv2.imshow("Depth", depth_img)
        # if seg_img is not None:
        #     cv2.imshow("Segmentation", seg_img)
        # cv2.waitKey(1)

        
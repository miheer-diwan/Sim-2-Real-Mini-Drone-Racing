from transitions import Machine
from transitions.extensions.states import add_state_features, Timeout
import time
from djitellopy import Tello
from threading import Thread, Lock
import cv2
from frame_handler import FrameHandler
import numpy as np
from scipy.spatial.transform import Rotation as R
from dual_quaternions import DualQuaternion
import math

# image = cv2.imread("/home/pear/drones/outputs/im.jpeg")
# cv2.imshow("Image",image)
# cv2.waitKey(0)

class FlyTello():
    def __init__(self):
        self.pose = None  # Placeholder for the drone's pose
        self.gate_count = 0  # Counter for the gates
        self.target_hover_height = 150  # Target hover height after takeoff
        self.height = 0
        self.connection_attempt = 0
        self.take_off_attempts = 0
        self.tello = Tello(retry_count=1)
        self.most_recent_frame = None
        self.streaming = False
        self.stream_thread = None
        self.gate_states=[[0.65,1.83,1.36,0.9816,0.,0.,0.1908],
                            [-0.55, 3.57, 1.36, 1., 0., 0., 0.],
                            [0.6, 5.59, 1.36, 0.9890, 0.,0.,0.1478]]
        self.frame_lock = Lock()
        self.frame_handler = FrameHandler(self.gate_states)
        self.current_camera_pose = None
        self.Q_W2Drone_current = DualQuaternion.from_quat_pose_array([1., 0., 0., 0., 0., 0., 1.6])

        # Initialize the state machine with the custom state machine class
        # self.machine = Machine(model=self, states=DroneStateMachine.states, transitions=DroneStateMachine.transitions, initial='Disconnected')

    def abort_mission(self):
        print("abort_mission")
        self.stop_streaming()
        self.tello.get_distance_tof()
        for i in range(3):
            self.tello.land()
        self.tello.end()
    
    # def process_frame(self):
    #     self.most_recent_frame = self.tello.get_frame_read().frame
    #     if self.most_recent_frame is not None and self.most_recent_frame.shape[0] == 720:
    #         # print(self.most_recent_frame.shape)
    #         self.frame_handler.handle_frame(self.most_recent_frame, self.gate_count)

    def stream_frame(self):
        self.streaming = True
        time.sleep(5.0)
        while self.streaming:
            with self.frame_lock:
                self.most_recent_frame = self.tello.get_frame_read().frame
            
            if self.most_recent_frame is not None and self.most_recent_frame.shape[0] == 720:
                # print(self.most_recent_frame.shape)
                self.frame_handler.handle_frame(self.most_recent_frame, self.gate_count)
            time.sleep(1/30)
    
    def stop_streaming(self):
        self.streaming = False
        # cv2.destroyAllWindows()
        self.stream_thread.join()

    def connect_tello(self):
        print("connect_tello")
        try:
            # ENTER COMMAND MODE AND TRY CONNECTING OVER UDP
            self.connection_attempt += 1
            print("Connect attempt ", self.connection_attempt)
            self.tello.connect()
            self.tello.set_video_fps(Tello.FPS_30)
            self.tello.streamon()
            time.sleep(5.0)
            self.tello.set_video_fps(Tello.FPS_30)
            print('Battery, ', self.tello.get_battery())
        except:
            print('Failed to connect or it connected but "ok" not received. Retrying...')
            if self.connection_attempt > 3:
                print('Failed to connect after multiple attempts')
                exit(-1)
            return False
        else:
            self.stream_thread = Thread(target=self.stream_frame)
            self.stream_thread.start()
            time.sleep(5.0)
            print("Connection successful")
            return True

    def check_take_off_success(self):
        self.height = self.tello.get_distance_tof()
        time.sleep(0.05)
        return self.height > 70.0
    
    def attempt_take_off(self):
        print("attempt_take_off")
        take_off_success = False
        while not take_off_success and self.take_off_attempts < 3:
            self.tello.takeoff()
            time.sleep(0.05)
            self.take_off_attempts += 1
            start_time = time.time()
            while True:
                el_time = time.time() - start_time
                if self.check_take_off_success():
                    # time.sleep(7)
                    current_attitude = self.tello.get_distance_tof()
                    try:
                        self.tello.move_up(self.target_hover_height-current_attitude)
                    except Exception as e:
                        print(e)
                        pass
                    print('Takeoff complete in seconds = ', el_time)
                    print('Altitude ', self.tello.get_distance_tof())
                    take_off_success = True
                    self.Q_W2Drone_current = DualQuaternion.from_quat_pose_array([1., 0., 0., 0., 0., 0., 1.6])
                    break
                elif el_time > 8.0:
                    break
                else:
                    # sleep for 1 second and check again
                    time.sleep(0.05)
    
    def initialize_pose(self):
        print("initialize_pose")
        self.height = 0
        pass

    def update_pose(self):
        print("update_pose")
        self.current_camera_pose = self.frame_handler.current_camera_pose
        Q_W2C = DualQuaternion.from_quat_pose_array(self.current_camera_pose)
        Q_C2drone = DualQuaternion.from_homogeneous_matrix(np.array([[0,-1,0,0],
                                                            [0,0,1,0],
                                                            [-1,0,0,0],
                                                            [0,0,0,1]]))
        self.Q_W2Drone_current = Q_W2C * Q_C2drone
        print("current camera_pose = ", self.current_camera_pose)

    # def find_gate(self):
    #     print("Finding gate")
    #     gate_found = False
    #     gate_centered = False
    #     center_thresh = 50
    #     # Rotatte towards next gate
    #     current_gate_state_xyz_wxyz = self.gate_states[self.gate_count]
    #     current_gate_state_xyz = current_gate_state_xyz_wxyz[0:3]
    #     current_gate_state_wxyz_xyz = current_gate_state_xyz_wxyz[3:7]

    #     current_gate_state_wxyz_xyz.extend(current_gate_state_xyz)

    #     # Q_W2G = DualQuaternion.from_quat_pose_array(current_gate_state_wxyz_xyz)
    #     # G_pos = Q_W2G.translation()
    #     # D_pos = self.Q_W2Drone_current.translation()
    #     # D2G_vec = np.array(G_pos) - np.array(D_pos)
    #     # Drone_x_vec = self.Q_W2Drone_current.homogeneous_matrix()[0:3,0]
    #     # angle_to_rotate = np.arccos(Drone_x_vec.dot(D2G_vec)/(np.linalg.norm(Drone_x_vec)*np.linalg.norm(D2G_vec)))
    #     # angle_to_rotate = int(angle_to_rotate * 180/math.pi)
    #     # print("angle_to_rotate", angle_to_rotate)
    #     # if angle_to_rotate > 0:
    #     #     self.tello.rotate_counter_clockwise(abs(angle_to_rotate))
    #     # elif angle_to_rotate < 0:
    #     #     self.tello.rotate_clockwise(abs(angle_to_rotate))

    #     while not gate_found or not gate_centered:
    #         closest_gate = self.frame_handler.closest_gate
    #         all_corner_present = self.frame_handler.closest_gate_all_corners_present
    #         if closest_gate is not None and all_corner_present:
    #             gate_found = True
    #         else:
    #             gate_found = False

    #         print(closest_gate)
    #         if closest_gate is not None:
    #             gate_center = closest_gate["gate_center"]
    #             img_center2gaet_center_hori = 480 - gate_center[0][0]
    #             img_center2gaet_center_vert = 360 - gate_center[0][1]

    #             if (img_center2gaet_center_hori > -center_thresh and img_center2gaet_center_hori < center_thresh):# and
    #             #     (img_center2gaet_center_vert > -center_thresh and img_center2gaet_center_vert < center_thresh)):
    #                 gate_centered = True
    #             else:
    #                 gate_centered = False
    #                 if img_center2gaet_center_hori < 0: #Yaw to the right
    #                     self.tello.rotate_clockwise(10)
    #                     # if img_center2gaet_center_vert > 0:
    #                     #     print("Yaw to the right and move up")
    #                     #     self.tello.send_rc_control(0,0,30,30) #Move up
    #                     # else:
    #                     #     print("Yaw to the right and move down")
    #                     #     self.tello.send_rc_control(0,0,-30,30) #Move down
    #                 else:
    #                     self.tello.rotate_counter_clockwise(10)
    #                     # if img_center2gaet_center_vert > 0:
    #                     #     print("Yaw to the left and move up")
    #                     #     self.tello.send_rc_control(0,0,30,-30) #Move up
    #                     # else:
    #                     #     print("Yaw to the left and move down")
    #                     #     self.tello.send_rc_control(0,0,-30,-30) #Move down
    #                 time.sleep(0.05)
    #         else:
    #             print("No Gate in view, yaw to the left")
    #             self.tello.send_rc_control(0,0,0,-30) #Yaw to the left
    #             time.sleep(0.01)
    #         time.sleep(0.01)
    #     print("Gate found and centered")
    #     self.tello.send_rc_control(0,0,0,0)
    #     time.sleep(2.0)
    #     self.tello.send_rc_control(0,0,0,0)
    #     self.update_pose()
    #     return gate_found

    def move_to_pose(self, gate_idx):
        at_pose = False
        print("Move to pose")
        if(gate_idx == 0):
            desired_pose_in_G = np.array([[0.2, -1.8, 0., 1.]]).T
        elif(gate_idx == 1):
            desired_pose_in_G = np.array([[0.2, -2.8, 0., 1.]]).T
        else:
            desired_pose_in_G = np.array([[0., -1., 0., 1.]]).T
        gate_quat_wxyz = np.array(self.gate_states[gate_idx][3:7])
        gate_quat_xyzw = np.roll(gate_quat_wxyz,-1)
        R_W2G = R.from_quat(gate_quat_xyzw)
        T_W2G = np.eye(4)
        T_W2G[0:3,0:3] = R_W2G.as_dcm()
        T_W2G[0:3,3] = np.array(self.gate_states[gate_idx][0:3])

        desired_pose_in_W = np.matmul(T_W2G, desired_pose_in_G).T
        Q_W2desired_gate_aligned = DualQuaternion.from_quat_pose_array(np.hstack((gate_quat_wxyz, desired_pose_in_W.flatten()[0:3])))
        # Q_W2C = DualQuaternion.from_quat_pose_array(self.current_camera_pose)
        # # Q_W2C = DualQuaternion.from_quat_pose_array([1., 0., 0., 0., 0., 0., 1.6])
        # Q_C2drone = DualQuaternion.from_homogeneous_matrix(np.array([[0,-1,0,0],
        #                                                     [0,0,1,0],
        #                                                     [-1,0,0,0],
        #                                                     [0,0,0,1]]))
        Q_G2drone = DualQuaternion.from_homogeneous_matrix(np.array([[0,-1,0,0],
                                                            [1,0,0,0],
                                                            [0,0,1,0],
                                                            [0,0,0,1]]))
        Q_W2Drone_desired = Q_W2desired_gate_aligned*Q_G2drone
        # Q_W2Drone_current = Q_W2C * Q_C2drone
        # Q_W2Drone_current = DualQuaternion.from_quat_pose_array([1., 0., 0., 0., 0., 0., 1.6])
        Q_Drone_current2Drone_desired = self.Q_W2Drone_current.inverse()*Q_W2Drone_desired
        
        current2desired_pose_array = Q_Drone_current2Drone_desired.quat_pose_array()
        current2desired_translation = current2desired_pose_array[4:7]
        current2desired_rotation_wxyz = current2desired_pose_array[0:4]
        T_current2desired = Q_Drone_current2Drone_desired.homogeneous_matrix()
        yaw_command = int(math.atan2(T_current2desired[1][0], T_current2desired[0][0])*180/math.pi)
        
        print("Current Drone State in World")
        print(self.Q_W2Drone_current.quat_pose_array())
        print("Desired Drone State in World")
        print(Q_W2Drone_desired.quat_pose_array())
        print("Desired pose in drone frame")
        print(current2desired_translation)
        print(current2desired_rotation_wxyz)
        x_command = int(current2desired_translation[0] * 100)
        y_command = int(current2desired_translation[1] * 100)
        z_command = int(current2desired_translation[2] * 100)
        print("x, y, z, yaw", x_command, y_command, z_command, yaw_command)
        try:
            self.tello.go_xyz_speed(x_command, y_command, 0, 30)
            time.sleep(0.05)
        except Exception as e:
            print(e)
            pass
        
        try:
            if yaw_command > 0:
                self.tello.rotate_counter_clockwise(abs(yaw_command))
            elif yaw_command < 0:
                self.tello.rotate_clockwise(abs(yaw_command))
            time.sleep(0.05)
        except Exception as e:
            print(e)
            pass
        # self.find_gate()
        # time.sleep(1.0)
        current_attitude = drone_sm.tello.get_distance_tof()
        try:
            if current_attitude > drone_sm.target_hover_height:
                drone_sm.tello.move_down(current_attitude-drone_sm.target_hover_height)
            else:
                drone_sm.tello.move_up(drone_sm.target_hover_height-current_attitude)
        except Exception as e:
            print(e)
            pass
        time.sleep(2.0)
        # self.find_gate()
        self.update_pose()
        current_gate_state_xyz_wxyz = self.gate_states[self.gate_count]
        current_gate_state_xyz = current_gate_state_xyz_wxyz[0:3]
        current_gate_state_wxyz_xyz = current_gate_state_xyz_wxyz[3:7]

        current_gate_state_wxyz_xyz.extend(current_gate_state_xyz)

        Q_W2G = DualQuaternion.from_quat_pose_array(current_gate_state_wxyz_xyz)
        G_pos = Q_W2G.translation()
        D_pos = self.Q_W2Drone_current.translation()
        D2G_vec = np.array(G_pos) - np.array(D_pos)
        Drone_x_vec = self.Q_W2Drone_current.homogeneous_matrix()[0:3,0]
        angle_to_rotate = np.arccos(Drone_x_vec.dot(D2G_vec)/(np.linalg.norm(Drone_x_vec)*np.linalg.norm(D2G_vec)))
        angle_to_rotate = int(angle_to_rotate * 180/math.pi)
        print("angle_to_rotate", angle_to_rotate)
        try:
            if angle_to_rotate < 0:
                self.tello.rotate_counter_clockwise(abs(angle_to_rotate))
            elif angle_to_rotate > 0:
                self.tello.rotate_clockwise(abs(angle_to_rotate))
            time.sleep(0.05)

        except Exception as e:
            print(e)
            pass
        
        q_delta_in_rotmat = R.from_euler('z', angle_to_rotate, degrees =True).as_dcm()
        homo_matrix = np.eye(4)
        homo_matrix[0:3,0:3] = q_delta_in_rotmat
        q_delta = DualQuaternion.from_homogeneous_matrix(homo_matrix)
        self.Q_W2Drone_current = self.Q_W2Drone_current * q_delta

    def fly_forward(self, distance_cm):
        try:
            self.tello.move_forward(distance_cm)
            time.sleep(0.05)
        except Exception as e:
            print(e)
            pass
        current_pose = self.Q_W2Drone_current 
        pose_to_add = DualQuaternion.from_homogeneous_matrix(np.array([[1, 0, 0, distance_cm/100],
                                                                    [0, 1, 0, 0],
                                                                    [0, 0, 1, 0],
                                                                    [0, 0, 0, 1]]))
        self.Q_W2Drone_current = current_pose * pose_to_add
        return True

    def increment_gate(self):
        self.gate_count += 1
        print(f"Passed through gate {self.gate_count}. Searching for the next gate.")

    def mission_complete(self):
        # Define the condition that determines if the mission is complete
        return self.gate_count >= 3


    def land_drone(self):
        # Logic to land the drone
        print("Landing the drone.")
        # Here you would implement the actual landing logic
        self.tello.land()

try:
    # Initialize the drone state machine
    drone_sm = FlyTello()

    drone_sm.connect_tello()

    # while True:
    #     time.sleep(0.2)

    # drone_sm.print_current_state()
    drone_sm.attempt_take_off()
    # drone_sm.arrive()
    # drone_sm.print_current_state(
    
    for gate_idx in range(3):
        drone_sm.move_to_pose(gate_idx)
        current_attitude = drone_sm.tello.get_distance_tof()
        try:
            if current_attitude > drone_sm.target_hover_height:
                drone_sm.tello.move_down(current_attitude-drone_sm.target_hover_height)
            else:
                drone_sm.tello.move_up(drone_sm.target_hover_height-current_attitude)
        except Exception as e:
            print(e)
            pass
        if gate_idx == 0:
            drone_sm.fly_forward(200)
        elif gate_idx == 1:
            drone_sm.fly_forward(240)
        else:
            drone_sm.fly_forward(200)

    # drone_sm.move_to_pose(gate_idx)
    # current_attitude = drone_sm.tello.get_distance_tof()
    # try:
    #     if current_attitude > drone_sm.target_hover_height:
    #         drone_sm.tello.move_down(current_attitude-drone_sm.target_hover_height)
    #     else:
    #         drone_sm.tello.move_up(drone_sm.target_hover_height-current_attitude)
    # except Exception as e:
    #     print(e)
    #     pass
    # drone_sm.fly_forward()

    # drone_sm.move_to_pose(gate_idx)
    # current_attitude = drone_sm.tello.get_distance_tof()
    # try:
    #     if current_attitude > drone_sm.target_hover_height:
    #         drone_sm.tello.move_down(current_attitude-drone_sm.target_hover_height)
    #     else:
    #         drone_sm.tello.move_up(drone_sm.target_hover_height-current_attitude)
    # except Exception as e:
    #     print(e)
    #     pass
    # drone_sm.fly_forward()

    # if drone_sm.state != 'Aborted':
    #     print("Taking off success!!!")
    #     drone_sm.print_current_state()

    #     drone_sm.start_mission()

    #     while drone_sm.state != 'Landed':
    #         k = cv2.waitKey(1) & 0xFF
    #         if k==ord('q'):
    #             drone_sm.tello.emergency()

    #         drone_sm.print_current_state()
    #         if drone_sm.state == 'Searching':
    #             # Drone is searching for the gate
    #             drone_sm.search()
    #         elif drone_sm.state == 'MovingPerpendicularToGate':
    #             drone_sm.move_to_pose()
    #         elif drone_sm.state == 'FlyingThroughGate':
    #             if drone_sm.mission_complete():
    #                 drone_sm.complete_mission()
    #             else:
    #                 drone_sm.flew_through()
    #         elif drone_sm.state == 'MissionComplete':
    #             # Mission is complete, time to land
    #             drone_sm.land()
    #         elif drone_sm.state == 'Landing':
    #             # Perform the landing
    #             drone_sm.land_drone()
    #             drone_sm.finish_landing()

    # At this point, the drone has landed
    print("Drone has landed successfully.")
except Exception as e:
    print(e)
    for i in range(5):
        drone_sm.tello.emergency()
    # drone_sm.abort_mission()
    drone_sm.tello.emergency()
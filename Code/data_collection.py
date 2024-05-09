from blender_bridge import BlenderBridge
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import json
import os
from shapely.geometry import Polygon
import random

def sample_spherical(npoints, radius, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    vec *= radius
    vec = vec.T
    return vec

def sample_hemi_spherical(npoints, radius, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    vec *= radius
    vec = vec.T
    vec[:,1] = np.negative(np.absolute(vec[:,1]))
    return vec

def sample_camera(npoints):
    x = -0.001*np.ones((npoints,1))
    y = -2*np.ones((npoints,1))
    z = np.zeros((npoints,1))

    positions = np.hstack((x,y,z))
    return positions

def stateAsH(state):
    R_matrix = R.from_quat([state[4],state[5],state[6],state[3]]).as_matrix()
    H = np.zeros((4,4))
    H[0:3,0:3] = R_matrix
    H[3,3] = 1
    H[0:3,3] = np.array(state[0:3]).T
    return H

def H_inv(H):
    rot = H[0:3,0:3]
    trans = H[0:3,3].reshape((3,1))
    H_inv = np.hstack((rot.T, -np.matmul(rot.T,trans)))
    H_inv = np.vstack((H_inv,np.array([0,0,0,1])))
    return H_inv

def transformation_multiply(H1,H2):
    R1 = H1[0:3,0:3]
    R2 = H2[0:3,0:3]
    T1 = H1[0:3,3]
    T2 = H2[0:3,3]
    H12 = np.hstack((np.matmul(R1,R2), (np.matmul(R2,np.matmul(R1,T1))+T2).reshape(3,1)))
    H12 = np.vstack((H12, np.array([0,0,0,1])))
    return H12

def calcGateCornerMarkersInCameraFrame(camera_state_W, gate_state_in_W):
    gate_corner_top_left = np.array([[-0.4, 0., 0.4, 1.0],
                                 [-0.1, 0., 0.4, 1.0],
                                 [-0.1, 0., 0.265, 1.0],
                                 [-0.265, 0., 0.265, 1.0],
                                 [-0.265, 0., 0.14, 1.0],
                                 [-0.4, 0., 0.14, 1.0]])
    gate_corner_top_right = gate_corner_top_left.copy()
    gate_corner_top_right[:,0] = -gate_corner_top_right[:,0]

    gate_corner_bottom_right = gate_corner_top_left.copy()
    gate_corner_bottom_right[:,0] = -gate_corner_bottom_right[:,0]
    gate_corner_bottom_right[:,2] = -gate_corner_bottom_right[:,2]

    gate_corner_bottom_left = gate_corner_top_left.copy()
    gate_corner_bottom_left[:,2] = -gate_corner_bottom_left[:,2]

    gate_corners_in_G = np.vstack((gate_corner_top_left,
                                   gate_corner_top_right,
                                   gate_corner_bottom_right,
                                   gate_corner_bottom_left))
    H_GinW = stateAsH(gate_state_in_W)
    H_CinW = stateAsH(camera_state_W)
    H_WinC = H_inv(H_CinW)
    # H_GinC = transformation_multiply(H_WinC,H_GinW)
    gate_corners_in_W = np.matmul(H_GinW, gate_corners_in_G.T)
    gate_corners_in_C = np.matmul(H_WinC, gate_corners_in_W).T

    return gate_corners_in_C

def calcGateCornersInCameraFrame(camera_state_W, gate_state_in_W):
    # gate_corner_top_left = np.array([[-0.4, 0., 0.4, 1.0],
    #                              [-0.1, 0., 0.4, 1.0],
    #                              [-0.1, 0., 0.265, 1.0],
    #                              [-0.265, 0., 0.265, 1.0],
    #                              [-0.265, 0., 0.14, 1.0],
    #                              [-0.4, 0., 0.14, 1.0]])
    # gate_corner_top_right = gate_corner_top_left.copy()
    # gate_corner_top_right[:,0] = -gate_corner_top_right[:,0]

    # gate_corner_bottom_right = gate_corner_top_left.copy()
    # gate_corner_bottom_right[:,0] = -gate_corner_bottom_right[:,0]
    # gate_corner_bottom_right[:,2] = -gate_corner_bottom_right[:,2]

    # gate_corner_bottom_left = gate_corner_top_left.copy()
    # gate_corner_bottom_left[:,2] = -gate_corner_bottom_left[:,2]

    # stripe_top = np.array([[-0.4, 0., 0.4, 1.0],
    #                         [0.4, 0., 0.4, 1.0],
    #                         [0.4, 0., 0.265, 1.0],
    #                         [-0.4, 0., 0.265, 1.0]])
    # stripe_bottom = stripe_top.copy()
    # stripe_bottom[:,2] = -stripe_bottom[:,2]

    # stripe_left = np.array([[-0.4, 0., 0.4, 1.0],
    #                         [-0.265, 0., 0.4, 1.0],
    #                         [-0.265, 0., -0.4, 1.0],
    #                         [-0.4, 0., -0.4, 1.0]])
    # stripe_right = stripe_left.copy()
    # stripe_right[:,0] = -stripe_right[:,0]
    # topleft_L = np.array([[-0.4, 0., 0.4, 1.0],
    #                         [0.4, 0., 0.4, 1.0],
    #                         [0.4, 0., 0.265, 1.0],
    #                         [-0.265, 0., 0.265, 1.0],
    #                         [-0.265, 0., -0.4, 1.0],
    #                         [-0.4, 0., -0.4, 1.0]])
    # bottomright_L = topleft_L.copy()
    # bottomright_L[:,0] = -bottomright_L[:,0]
    # bottomright_L[:,2] = -bottomright_L[:,2]

    # gate_corners_in_G = np.vstack((gate_corner_top_left,
    #                                gate_corner_top_right,
    #                                gate_corner_bottom_right,
    #                                gate_corner_bottom_left))
    # gate_stripes_in_G = np.vstack((stripe_top,
    #                                stripe_bottom,
    #                                stripe_left,
    #                                stripe_right))
    # gate_Ls_in_G = np.vstack((topleft_L,bottomright_L))
    gate_size = 0.8
    gate_corners_in_G = np.array([[-gate_size/2,0.0,gate_size/2,1.0],
                                 [gate_size/2,0.0,gate_size/2,1.0],
                                 [gate_size/2,0.0,-gate_size/2,1.0],
                                 [-gate_size/2,0.0,-gate_size/2,1.0]])
    H_GinW = stateAsH(gate_state_in_W)
    H_CinW = stateAsH(camera_state_W)
    H_WinC = H_inv(H_CinW)
    # H_GinC = transformation_multiply(H_WinC,H_GinW)
    gate_corners_in_W = np.matmul(H_GinW, gate_corners_in_G.T)
    gate_corners_in_C = np.matmul(H_WinC, gate_corners_in_W).T

    return gate_corners_in_C

def get_corner_markers_on_image(camera_state, gate_state, K):
    gate_corners_in_C = calcGateCornerMarkersInCameraFrame(camera_state,gate_state)
    # H_CinW = stateAsH(camera_state)
    # H_WinC = H_inv(H_CinW)
    # RT = H_CinW[0:3,:]
    R = np.array([[1, 0, 0],
                  [0, -1, 0],
                  [0, 0, -1]])
    # RT = np.hstack((np.identity(3), np.array([0,0,0]).reshape((3,1))))
    RT = np.hstack((R, np.array([0,0,0]).reshape((3,1))))
    corners_on_image = np.matmul(K,np.matmul(RT,gate_corners_in_C.T))
    corners_on_image = corners_on_image/corners_on_image[2,:]
    corners_on_image = corners_on_image[0:2,:].T
    # corners_on_image = corners_on_image.reshape((2,4,2))
    corners_on_image = corners_on_image.reshape((4,6,2))
    return corners_on_image

def get_corners_on_image(camera_state, gate_state, K):
    gate_corners_in_C = calcGateCornersInCameraFrame(camera_state,gate_state)
    # H_CinW = stateAsH(camera_state)
    # H_WinC = H_inv(H_CinW)
    # RT = H_CinW[0:3,:]
    R = np.array([[1, 0, 0],
                  [0, -1, 0],
                  [0, 0, -1]])
    # RT = np.hstack((np.identity(3), np.array([0,0,0]).reshape((3,1))))
    RT = np.hstack((R, np.array([0,0,0]).reshape((3,1))))
    corners_on_image = np.matmul(K,np.matmul(RT,gate_corners_in_C.T))
    corners_on_image = corners_on_image/corners_on_image[2,:]
    corners_on_image = corners_on_image[0:2,:].T
    # corners_on_image = corners_on_image.reshape((2,4,2))
    corners_on_image = corners_on_image.reshape((1,4,2))
    return corners_on_image

def get_random_gate(x_min,x_max,y_min,y_max,z_min,z_max):
    location_x = np.random.uniform(x_min,x_max)
    location_y = np.random.uniform(y_min,y_max)
    location_z = np.random.uniform(z_min,z_max)

    location = np.array([location_x,location_y,location_z])
    orientation = R.random().as_quat()
    state = np.hstack((location, orientation))
    # state = np.hstack((location, [1.0,0.0,0.0,0.0]))
    # state = np.array([0.5,0.0,0.5,1.0,0.0,0.0,0.0])
    return state

def find_intersections(four_corners_on_image):    
    # creating polygons using Polygon()
    intersections = []
    for corner in four_corners_on_image:
        corner_poly = Polygon(corner) 
        gate = Polygon([(0, 0), (720, 0), (720, 720), (0, 720)]) 
        
        # using intersection() 
        gate_inside_img = gate.intersection(corner_poly)
        if gate_inside_img.type!='MultiPolygon':
            intersection = np.asarray(gate_inside_img.exterior.coords)
        else:
            intersection = np.array([])

        if intersection.shape[0] >= 3:
            intersections.append(intersection)
        else:
            intersections.append(np.array([]))

    return intersections

blender_bridge = BlenderBridge()

gate_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
gate_position = np.array([0.0,0.0,0.0])
gate_states=[[0.0,0.0,0.0,1.0,0.0,0.0,0.0]]
#sample 100 camera locations on 5m radius sphere
sample_size = 5000
# camera_positions = sample_hemi_spherical(sample_size, radius = 3)
# camera_positions = np.array([[-0.001,-1,0]])
camera_positions = sample_camera(sample_size)
camera_z_original = np.array([[0., 0., -1.]])
camera_z_directions = gate_position - camera_positions
camera_quaternions_wxyz = []
for camera_z_direction in camera_z_directions:
    camera_rotation,_ = R.align_vectors(np.reshape(camera_z_direction,(1,-1)),camera_z_original)
    camera_quaternion_xyzw = camera_rotation.as_quat()
    # camera_quaternion_xyzw = [0,0,0,1]
    camera_quaternion_wxyz = np.roll(camera_quaternion_xyzw, 1)
    camera_quaternions_wxyz.append(camera_quaternion_wxyz.tolist())

camera_states = np.hstack((camera_positions, camera_quaternions_wxyz))




camera_data_list = []

render_depth = False
render_seg=False
render_opt=False
image_width = 720
image_height = 720
K = np.array([[1384.3, 0.000000, 360],
            [0.000000, 1383.9, 360],
            [0.000000, 0.000000, 1.000000]])


for idx ,camera_state in enumerate(camera_states):
    print(idx)
    gate_states = []
    gate_corners_on_image = []
    gate_corner_markers_on_image = []
    for i in range(3):
        gate_state = get_random_gate(-1,1,0,5,-1,1)
        gate_states.append(gate_state.tolist())
        # four_stripes_on_image = get_corners_on_image(camera_state,gate_state,K)
        # four_stripes_on_image = find_intersections(four_stripes_on_image)
        four_corners_on_image = get_corners_on_image(camera_state,gate_state,K)
        four_corners_on_image = find_intersections(four_corners_on_image)
        for corners in four_corners_on_image:
            gate_corners_on_image.append(corners)
        
        four_corner_markers_on_image = get_corner_markers_on_image(camera_state,gate_state,K)
        four_corner_markers_on_image = find_intersections(four_corner_markers_on_image)
        for corner_marker in four_corner_markers_on_image:
            gate_corner_markers_on_image.append(corner_marker)
        

    
    if idx == 0:
        RGBA_img, depth_img, seg_img = blender_bridge.getRender([camera_state.tolist()], gate_states,
                                        render_depth, render_seg,render_opt,
                                        reset_animation = True,
                                        change_camera_setting = True,
                                        image_width=image_width,
                                        image_height=image_height,
                                        K = K.tolist())
    else:
        RGBA_img, depth_img, seg_img = blender_bridge.getRender([camera_state.tolist()], gate_states,
                                        render_depth, render_seg,render_opt,
                                        reset_animation = False,
                                        change_camera_setting = False,
                                        image_width=image_width,
                                        image_height=image_height,
                                        K = K.tolist())
    
    # for gate_corners in gate_corners_on_image:
    #     random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), 255)
    #     # random_color.append(255)
    #     for corner in gate_corners:
    #         x, y = corner.ravel()
    #         cv2.circle(RGBA_img, (int(x), int(y)), 5, random_color, -1)
    
    # for gate_corners in gate_corner_markers_on_image:
    #     random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), 255)
    #     # random_color.append(255)
    #     for corner in gate_corners:
    #         x, y = corner.ravel()
    #         cv2.circle(RGBA_img, (int(x), int(y)), 5, random_color, -1)

    camera_data_dict = {
        "position": {
            "x": camera_state[0],
            "y": camera_state[1],
            "z": camera_state[2]
        },
        "quaternion": {
            "w": camera_state[3],
            "x": camera_state[4],
            "y": camera_state[5],
            "z": camera_state[6]
        }
    }

    base_path = '/home/oliver/GateDataset/outputs'
    if not os.path.exists(base_path):
        os.makedirs(base_path)
    if not os.path.exists(base_path+'/RGBA'):
        os.makedirs(base_path+'/RGBA')
    if not os.path.exists(base_path+'/depth'):
        os.makedirs(base_path+'/depth')
    if not os.path.exists(base_path+'/segment'):
        os.makedirs(base_path+'/segment')
    if not os.path.exists(base_path+'/labels'):
        os.makedirs(base_path+'/labels')
    RGBA_path = base_path+'/RGBA/rgba_img_'+str(idx)+'.png'
    depth_path = base_path+'/depth/depth_img_'+str(idx)+'.png'
    seg_path = base_path+'/segment/seg_img_'+str(idx)+'.png'
    labels_path = base_path+'/labels/rgba_img_'+str(idx)+'.txt'
    cv2.imwrite(RGBA_path,RGBA_img)
    camera_data_dict["RGBA_path"] = RGBA_path
    if depth_img is not None:
        cv2.imwrite(depth_path,depth_img)
        camera_data_dict["depth_path"] = depth_path
    if seg_img is not None:
        cv2.imwrite(seg_path,seg_img)
        camera_data_dict["seg_path"] = seg_path

    # Specify the file path where you want to create or write to the file

    # Open the file in write mode
    with open(labels_path, "w") as file:
        for idx,line in enumerate(gate_corners_on_image):
            if line.shape[0] > 0:
                line = np.round(line.flatten()/720,3)
                line_as_string = '0 ' + ' '.join(map(str, line))
                # Convert elements to strings and join them with spaces
                # if idx%4 == 0 or idx%4 == 1:
                #     line_as_string = '0 ' + ' '.join(map(str, line))
                # else:
                #     line_as_string = '1 ' + ' '.join(map(str, line))
            # Write the line to the file
            file.write(line_as_string + '\n')
        for idx,line in enumerate(gate_corner_markers_on_image):
            if line.shape[0] > 0:
                line = np.round(line.flatten()/720,3)
                line_as_string = '1 ' + ' '.join(map(str, line))
            # Write the line to the file
            file.write(line_as_string + '\n')

    
    

    camera_data_list.append(camera_data_dict)

if not os.path.exists(base_path+'/gt_pose'):
        os.makedirs(base_path+'/gt_pose')
with open(base_path+'/gt_pose/camera_data_new.json', 'w') as json_file:
    json.dump(camera_data_list, json_file, indent=4)


import bpy
import os
import cv2
import random
import math
import importlib
import numpy as np

import frame_utils as frame
importlib.reload(frame)

def setRenderSettings():
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'
    bpy.context.scene.eevee.taa_render_samples = 6 # Lowering this makes rendering fast but noisy
    bpy.context.scene.eevee.taa_samples = 6
    bpy.context.scene.render.resolution_x = 320 # Reduce to speed up
    bpy.context.scene.render.resolution_y = 240 # Reduce to speed up

    
def render():    
    # DO NOT CHANGE THIS FUNCTION BELOW THIS LINE!        
    path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

    # Render Second Camera for Third Person view of the Drone
    # cam = bpy.data.objects['FollowViewCam']    
    # bpy.context.scene.camera = cam
    # bpy.context.scene.render.filepath = os.path.join(path_dir, 'FollowViewCam', 'Frame%04d'%(bpy.data.scenes[0].frame_current))
    # bpy.ops.render.render(write_still=True)

    # Render Drone Camera
    cam = bpy.data.objects['DownCam']    
    bpy.context.scene.camera = cam
    bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam', 'Frame%04d'%(bpy.data.scenes[0].frame_current))
    bpy.ops.render.render(write_still=True)
    
# def visionAndPlanner(GoalLocation):
#     # USE cv2.imread to the latest frame from 'Frames' Folder
#     # HINT: You can get the latest frame using: bpy.data.scenes[0].frame_current
#     # USE ReadEXR() function provided below to the latest depth image saved from 'Depth' Folder
#     # Compute Commands to go left and right (VelX) using any method you like
#     path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path
#     print(os.path.join(path_dir, 'Frames', 'Frame%04d'%(bpy.data.scenes[0].frame_current)))
#     I = cv2.imread(os.path.join(path_dir, 'Frames', 'Frame%04d.png'%(bpy.data.scenes[0].frame_current)))
#     D = Exr2Depth(os.path.join(path_dir, 'Depth', 'Depth%04d.exr'%(bpy.data.scenes[0].frame_current)))

#     # You can visualize depth and image as follows
#     # cv2.imshow('Depth', D)
#     # cv2.imshow('Image', I)
#     # cv2.waitKey(0)
#     VelX = math.copysign(0.5, 0.5-random.uniform(0, 1))
#     return VelX

def addKeyFrameForObject(Object, Frame):
    Object.keyframe_insert(data_path="location", frame = Frame)
    Object.keyframe_insert(data_path="rotation_quaternion", frame = Frame)
    #!!!!!!!!!!
    #keyframe_insert has to be called before frame_set, otherwise the location won't update correctly! Don't know why.
    #!!!!!!!!!!
    
def advanceOneFrame():
    bpy.data.scenes['Scene'].frame_set(bpy.data.scenes['Scene'].frame_current+1)
    
# def moveDrone(Drone):
    
#     MaxFrame = 100
    
#     while(bpy.data.scenes['Scene'].frame_current<MaxFrame):
#         render()
# #        print(Drone.location)
#         Drone.location[1] += 0.5
#         addKeyFrameForObject(Drone, bpy.data.scenes['Scene'].frame_current)
#         advanceOneFrame()

def stepBlender(current_ned_state):
    BlenderDrone = bpy.data.objects['com_frame']
    # TODO clean up a bit and convert in a more general fashion

    # current_enu_state = frame.ned2enu_fullstate(current_ned_state)
    xyz_ned = current_ned_state[0:3]

    # convert to ENU and set location of blender object
    BlenderDrone.location[0]=xyz_ned[0]
    BlenderDrone.location[1]=-xyz_ned[1]
    BlenderDrone.location[2]=-xyz_ned[2]

    quat_NED_wxyz = current_ned_state[6:10]
    # since we are changing the basis from i_ned, j_ned, k_ned to i_enu, j_enu, k_enu
    #   and we know the relationship between the basis, we can simply replace the j and k component with their negation. I hope it works
    quat_enu_wxyz = np.array((quat_NED_wxyz[0], quat_NED_wxyz[1], -quat_NED_wxyz[2], -quat_NED_wxyz[3]))
    BlenderDrone.rotation_quaternion = quat_enu_wxyz

    addKeyFrameForObject(BlenderDrone, bpy.data.scenes['Scene'].frame_current)
    advanceOneFrame()

    # render()

def init():
    setRenderSettings()
    # Reset Frame to 0
    bpy.data.scenes['Scene'].frame_set(0)
    # Deselect all objects
    scene = bpy.context.scene
    scene.animation_data_clear()
    for o in scene.objects:
        o.animation_data_clear()
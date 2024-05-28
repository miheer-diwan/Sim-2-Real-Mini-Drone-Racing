# Sim-2-Real Mini Drone Racing

## This project is a part of ["RBE 595: Hands-On Autonomous Aerial Robotics"](https://pear.wpi.edu/teaching/rbe595/fall2023.html) taught by [Prof. Nitin Sanket](https://nitinjsanket.github.io/index.html) in Fall 2023 at Worcester Polytechnic Institute

### Introduction: 
The task was to autonomously detect a square window and fly through the center of the window in a given environment autonomously. 


**The catch:** The training for the detection model had to be done entirely on simulated data.

Check the Report for an in-depth explanation: [Report.pdf](Report.pdf)


### The problem can be divided into four parts:
1. Data Generation
2. Data Augmentation and Training a DL network
3. Deployment and Testing
4. Window Pose Estimation

### Detailed Explanation for each task:

**1. Data generation and Augmentation**
- For this task, I created a model of the window in Blender. By randomly spawning the window in different poses I could capture images to create my training data set. 

- Since the model was created in Blender, and I had the coordinates of the 4 corners of the window, I could easily extract them and use these as my ground truth labels and masks.

- Since there is a square-shaped hole in the window, we used the "Domain Randomization" technique to add different backgrounds in each image to create a diverse dataset. For this, we used the "Flying chairs" dataset and created 3000 images for our training set.

**2. Data Augmentation and Training a DL model**
- For data augmentation, I used Roboflow for adding noise, rotations, and random crops to the images and expanded the original training dataset from 3000 to 24000.

- Using PyTorch and the YOLOv8 instance segmentation model, I trained our model to segment windows in the real world.

**3. Deployment and Testing:**
- Our drone was a DJI TelloEDU and we were using NVIDIA's JETSON Orin Nano as a computer. Even though we were using the YOLOv8-nano model, it was still too big as we were running multiple networks in parallel. We converted our segmentation model using Tensor RT SDK to optimize the runtime in real-time.

**4. Pose Estimation:**
- After testing the model on frames captured from the drone's camera, we got accurate segmentation masks and the four corners for the windows in the environments. The next step was to use OpenCV's SolvePnP function to get the pose in real world so the drone could be commanded to fly through the gate.

### YOLOv8 Instance Segmentation Output on Drone Camera Frames
https://github.com/miheer-diwan/Sim-2-Real-Mini-Drone-Racing/assets/79761017/7def7f6c-596a-4034-b03f-d7c7821fca13

### Pose Estimation Output with Blender Environment
https://github.com/miheer-diwan/Sim-2-Real-Mini-Drone-Racing/assets/79761017/80f1c210-ef7b-489a-be07-30a9df0560e6

### Video Run:
https://github.com/miheer-diwan/Sim-2-Real-Mini-Drone-Racing/assets/79761017/3124f752-78dd-4c63-9f96-9989577236c7





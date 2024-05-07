# Courses and Course Projects

## AER1216 - Fundamentals of Unmanned Aerial Vehicles

Course for learning UAVs fundamentals for fixed-wing and multirotor UAVs. Involves creating MATLAB programs and simulations for fixed-wing and multirotor propellers, calculating performance, and creating performace charts. Involves Simulink modelling for multirotor and fixed-wing UAVs.  

## AER1217 - Development of Autonomous Unmanned Aerial Vehicles

Course for learning multirotor UAV software algorithms. Involves doing multiple laboratory projects that build into a quadrotor racing final project. Final project includes developing a RRT* based algorithm to pass through gates and avoid obstacles, testing on a PyBullet drone simulator, and testing on Crazyflie drones in real life. Laboratories include programming drones to fly to waypoints and flying in circular trajectories in a PyBullet drone simulator, vision-based landmark state estimation from drones using OpenCV algorithms, and visual odometry based localization of a cyclist's trajectory using scalar based point cloud alignment techniques, RANSAC, and OpenCV feature detectors. Majority of the programming is in Python.

## AER1513 - State Estimation for Aerospace Vehicles

An in depth course on state-of-the-art state estimation techniques for robots. Course involves doing three in depth laboratories on different state estimation scenarios. The first scenario is a 1D state estimation problem, determining the distance a mobile robot is from a wall, being solved using linear batch state estimation. The second problem is a 2D state estimation problem, determining the position of a mobile robot on flat ground using obstacles as visible landmarks, solved by using an extended Kalman filter. The third problem is a 3D state estimation problem, determining the position of a hand-held stereo camera in 3D observing landmarks on the ground, solved by using full-batch nonlinear estimation techniques. Programming uses a combination of MATLAB and Python. 

## AER1515 - Perception for Robotics

A broad course on state-of-the-art perception techniques for robotics. The course involves doing three assignments on robotic perception algorithms and doing a robotic perception final project. The first assignment involved creating an animal face classification convolutional neural network. The second assignment involved using feature detection and matching algorithms from OpenCV, and aligning point clouds using iterated closest point (ICP) algorithm. The third assignment uses the KITTI dataset to perform depth estimation, using YOLO for object detection in order to do segmentation. The final project uses aerial images from drones and the goal is to segment the images into different landmark types. State-of-the-art segmentation networks like DeepLabV3 is used to perfrom segmentation. The purpose of the project is to implement the algorithm on drones in order to for the drones to plant tree seeds in optimal areas for growth. Programming is in Python.

## AER1516 - Robot Motion Planning

A course on robot motion planning algorithms. The course consists of two programming assignments and a final project. The first programming assignment uses Dubins path to implement an RRT* path planner for robots. The second assignment uses an extended Kalman filter (EKF) to perform state estimation of a mobile robot travelling in a figure eight. The final project involves training a drone in a Unity based simulator AirSim to avoid obstacles in a simulation environment using state-of-the-art reinforcement techniques including using Deep Q-Networks (DQNs), policy gradients, and the TD3 algorithm. Programming is done in Python.

## APS360 - Artificial Intelligence Fundamentals

A course on machine learning fundamentals. Content includes doing many labs on artificial neural networks (ANNs), convolutional neural networks (CNNs), recurrent neural networks (RNNs), autoencoders, and generative adversarial networks (GANs). Gesture recognition and spam detection are example labs. Final project involved using an ANN to analyze hockey statistics to create NHL draft rankings based on data from previous years. Programming is done in Python, using PyTorch for machine learning.

## Capstone - Drone Capstone Project

A capstone project in a group of 4 to build and design a drone from scratch to perform several tasks critical from nuclear safety inspections. Tasks in order are building the drone, making the drone hover in place 1m from the ground, making the drone travel to waypoints, and to autonomously avoid coloured obstacles by turning in certain directions. Drone is equipped with a computer vision object detection algorithm (YoloV5), visual SLAM for accurate localization using visual inertial odometry (VIO) and trajectory rollout based RRT* motion planning. Projects involves heavy use of ROSPy and ROSCPP, RViz and Gazebo for drone simulation, and code written in Python and C++ programming languages. Majority of code is not public.

## CSC412 - Probabilistic Learning and Reasoning

Course involves complex mathematical probabilistic learning and reasoning concepts and applications in programming. Involves three programming assignments. The first assignment is about decision theory and a naive Bayes generative model for fashion MNIST dataset. The second assignment is about image denoising using the Loopy BP algorithm and Markov chain Hamiltonian Monte Carlo in the TrueSkill model. The third assignment is about Stochastic Variational Inference in the TrueSkill model and training a variational autoencoder (VAE) on synthetic data. Programming is done in Python.

## CSC413 - Neural Networks and Deep Learning

Course taught by Adam Optimizer publisher Jimmy Ba, teaching state-of-the-art neural network and deep learning techniques. Course consisted of a series of assignments and a final project. Assignments involves creating convolutional neural networks (CNNs), creating attention based transformers for large language modelling, exploring Glove embeddings, BERT, and CLIP networks, developing Graph Attention (GAT), Graph Convolution (GCN), and Deep Q Networks (DQNs), and using large language models (LLMs). Final project involved using CNNs and transfer learning to identify COVID in patients with X-ray images. Programming is done in Python.

## ECE470 - Robot Modelling and Control 

Course about modelling and controlling manipulator robots. Majority of course involved doing written homework problems but a portion of the course involved doing practicals with KUKA and PUMA560 robots. One practical involved simulating a PUMA560 robot in MATLAB using the Robotics Toolbox by implementing the forward and inverse kinematics.

## ECE557 - Linear Control Theory

Course about linear control theory which included doing written homework problems and multiple practicals about balancing an inverted pendulum on a moving cart. Majority of linear control theory calculations, modelling, and plotting is done using MATLAB and Simulink. 

## ECE1647 - Introduction to Nonlinear Control Systems

Course about nonlinear control systems which included doing written homework problems and a final project and report. The final project involved exploring nonlinear control techniques for drone control including feedback linearization, integral backstepping, and sliding mode control. Exploration about the techniques were done in Simulink. 

## ECE1658 - Geometric Nonlinear Control of Robotics

Advanced course about nonlinear control of walking robots using Virtual Holonomic Constraints (VHCs). The course involves a series of assignments with a final project. One assignment involved simulating a five-link, six-joint, 2D walking robot with feedback linearization based control in MATLAB. One assignment involves simulating a bicycle model robot following a circular trajectory due to stabilizing the zero dynamics manifold. The final project involved simulating an acrobot walking gait to perform stable walking in MATLAB both using preexisting parameters and finding optimal parameters using an optimization formulation and solving the optimization problem using MATLAB fmincon. The final project also involved a creative portion simulating an acrobot walkover gait.   

## ROB501 - Computer Vision for Robotics

Course is about different computer vision techniques used in robotics applications. The course involves multiple programming assignments involving homography techniques, landmark and camera pose estimation, stereo vision depth map generation, visual servoing, and determining the direction of the sun using convolutional neural networks (CNNs). Course also included a hackathon to create a panorama using feature detection and matching with OpenCV. Programming is done in Python.

## ROB521 - Mobile Robotics and Perception

Course is about advanced concepts about mobile robotics and includes both programming assignments in MATLAB and doing practicals with TurtleBots and programming using ROSPy. Programming assignments involve coding a wheel odometry algorithm and building a map using wheel odometry, coding a occupancy grid algorithm and a particle filter to localize within a known map, and implementing a probabilistic roadmap (PRM) algorithm to generate paths in a maze to solve using the A* algorithm. Labs involve navigating a TurtleBot through a maze using RRT* and calculating the wheel radius of the TurtleBot using dead reckoning. 

## Thesis - Deformable Image Registration for Image Guided Robotics

The thesis project is about deformable image registration for image guided robotics. The project involves integrating a Franka Emika Panda robot with open source medical imaging software 3D Slicer using ROS2. In addition, there is a camera streaming images of a deformable model to the medical imaging software and a model of the robot is loaded into the software. The deformable model image is registered to the robot using a deformable image registration neural network Voxelmorph. All the code involving 3D Slicer is written in C++ and RCLCPP and the code for controlling the robot uses the C++ MoveIt library. The code for image registration and streaming is written in Python and RCLPY. Additional research was performed in the summer, implementing rigid registration instead of deformable image registration using the same setup by tracking Aruco markers in the streamed images for higher accuracy.

# Programming Languages

Python, MATLAB, C++
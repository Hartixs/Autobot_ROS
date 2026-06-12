# Robocon 2026: Robominds

![ROS2](https://img.shields.io/badge/ros2-jazzy-blue) ![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange) ![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-yellow)

Welcome to my R&D repo for Robocon 2026. I've spent a lot of time testing different navigation and simulation stacks to figure out what actually works for our competition bot. 

If you're a junior picking this up, or a recruiter checking out my work, this README breaks down the final tech stack, what I tested, what failed, and how to get it running.

🎥 **[Insert GIF of the robot navigating in Gazebo Harmonic or real-world testing here]**

---

## The Tech Stack
* **Framework:** ROS 2 Jazzy
* **Simulation:** Gazebo Harmonic
* **Mapping:** `slam_toolbox`
* **Navigation:** `nav2`
  * **Global Planner:** Smac Planner
  * **Local Planner:** MPPI Planner
* **Hardware Profile:** Built for a Raspberry Pi 5 / Jetson Nano setup using an RPLIDAR A1.

---

## R&D Log: What I tried (and what actually worked)

### 1. Migrating to Gazebo Harmonic
I originally started testing with an articulated bot model built for ROS Foxy and Gazebo Classic. Since those are basically ancient history now, I scrapped that and ported the entire URDF/SDF setup over to ROS 2 Jazzy and Gazebo Harmonic. It took some work to rewrite the integration, but it's much more future-proof and handles the physics steps way better.

### 2. VSLAM vs. 2D SLAM
I really wanted to get RTAB-map working for 3D Visual SLAM, but it completely failed in simulation. Default Gazebo worlds are heavily textureless, so the camera couldn't grab enough visual keypoints and the odometry drifted almost instantly. 

I ended up dropping back to standard 2D LiDAR SLAM. I tested Google Cartographer for a bit, but ultimately went with `slam_toolbox` because its lifelong mapping capabilities are rock solid and it integrates with Nav2 much smoother.

### 3. Tuning the Nav2 Stack
Out-of-the-box Nav2 parameters just weren't cutting it for the speeds and dynamic environments we need for Robocon. 
* **Keepout Zones:** I set up keepout filters using the costmap filter server to strictly block the global planner from generating paths through restricted competition zones.
* **Swapping Planners:** I ditched the default NavFn global planner for the **Smac Planner** to get paths that actually make sense for our kinematics. For the local planner, I swapped DWB for **MPPI**, which gives us much better predictive collision avoidance and smoother trajectories when running at higher speeds.

---

## Getting Started (For Juniors)

If you're taking this over or want to run the sim yourself, here's how to set it up.

### Prerequisites
You need to be on Ubuntu 24.04 with ROS 2 Jazzy installed. Run this to grab the specific packages we rely on:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

Here are all the other commands that I had used in launching various stacks

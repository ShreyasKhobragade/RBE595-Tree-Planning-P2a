<div align="center">
  
# Tree Planning Through The Trees! (P2a)
### Hands-On Aerial Robotics [RBE595]

[![Path Planning](https://img.shields.io/badge/Planning-RRT*-orange.svg)](#)
[![Control](https://img.shields.io/badge/Control-Cascaded%20PID-blue.svg)](#)
[![Framework](https://img.shields.io/badge/Language-Python-green.svg)](#)

*Implementation of motion planning algorithms (RRT*, Spline Trajectories) and a Cascaded PID Controller for autonomous 3D quadrotor navigation in dense environments.*

</div>

---

## 📖 Overview

This repository holds Project 2a (P2a) of the *Hands-On Aerial Robotics* course. This project ventures into full autonomous motion planning by tasking a simulated quadrotor to navigate from a start position to a goal position through a completely mapped 3D environment with obstacles.

### Key Algorithmic Components
Navigating the environment accurately requires a robust, 4-step software stack:
1. **Environment Parser:** Transforms rectangular boundary inputs from `.txt` maps into bounding volume logic with robust line-collision and point-collision subroutines.
2. **Global Path Planner (RRT*):** An optimally-rewiring Rapidly-exploring Random Tree that generates feasible, collision-free node paths from start to goal.
3. **Trajectory Generation:** Converts the sharp node transitions of RRT* into dynamically feasible, cubic/quintic continuous spline trajectories respecting velocity vectors.
4. **Cascaded PID Controller:** Dynamically follows the generated trajectories on the fly using a multi-loop PID controller enforcing zero-tolerance obstacle avoidance.

---

## 🚀 Execution & Methodology

### 1. Planning with RRT*
The Rapidly Explorer Random Tree Star (RRT*) algorithms intelligently samples the known environment. It actively expands the tree towards the goal while optimally rewiring local nodes to strictly minimize total distance paths, avoiding the cuboidal boundaries extracted by the environment parser.

### 2. Trajectory Generation & Smoothing
Raw RRT* nodes are joined and evaluated. A continuous spline trajectory is then generated. The speed algorithm strictly honors the robot's physical dimensions padded by a variable `safety_margin`, aggressively checking points against 3D boundaries to reject invalid routes immediately upon prediction.

### 3. Cascaded Drone Control
The quadrotor incorporates a robust inner/outer PID sequence influenced by standard flight-stacks:
- **Outer Loop (Position):** Controls the XYZ bounds and sets velocity waypoints.
- **Inner Loop (Velocity):** Enforces rate control maintaining velocity over constraints.

---

## 🚀 Quick Start & Usage

### Requirements
```bash
pip install numpy scipy matplotlib
```

### Usage
```bash
# Open the source directory
cd src/

# Run the simulation 
python run_sim.py
```

### Outputs
- **Result Output:** Outputs 3D visualizations generated utilizing Matplotlib.
- **Assets:** Included `.mp4` video renderings of the successful simulations navigating the blocked environments dynamically are located in the `assets/` directory.

---

## 📁 Repository Structure

```text
RBE595-Tree-Planning-P2a/
├── src/                   # Python sources containing planners, controls, and environment logic
├── maps/                  # Environment files indicating obstacle geometries (.txt)
├── Report.pdf             # Exhaustive formulation and implementation findings
├── assets/                # Rendered video graphs of the simulation 
└── README.md              # Project overview
```

# Self-Driving Car Control Systems

Interactive visualizations demonstrating control systems algorithms applied to autonomous tracking. Watch a red ball intelligently chase a blue ball using two fundamental approaches: **Kalman Filter** (optimal state estimation) and **PID Controller** (feedback control).

## 🎯 Overview

This project provides educational simulations of autonomous tracking algorithms, commonly used in:
- Self-driving vehicles
- Robotics and drone navigation
- Target tracking systems
- Industrial automation

Both implementations simulate a "hunter" (red ball) pursuing a "target" (blue ball) moving in a circular path, showcasing different algorithmic approaches to the same problem.

## 📁 Project Structure

### HTML Visualizations (Browser-Based)
- **`kalman_ball_chase.html`** - Kalman Filter implementation with predictive state estimation
- **`pid_ball_chase.html`** - PID Controller implementation with feedback control

### Jupyter Notebooks (Python-Based)

#### Core Demonstrations
- **`kalman_ball_chase.ipynb`** - Interactive notebook with Kalman Filter animation
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/kalman_ball_chase.ipynb)

- **`pid_ball_chase.ipynb`** - Interactive notebook with PID Controller animation
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/pid_ball_chase.ipynb)

#### Course Notebooks (15-Week Curriculum)

| Module | Week | Topic | Open in Colab |
|--------|------|-------|---------------|
| **Module I: Introduction & Foundations** | 1 | Introduction to Autonomy | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week01_introduction_to_autonomy.ipynb) |
| | 2 | Vehicle Dynamics & Kinematics | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week02_vehicle_dynamics_kinematics.ipynb) |
| | 3 | Control Theory & Actuation | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week03_control_theory_actuation.ipynb) |
| **Module II: Perception & Localization** | 4 | Sensor Technologies & Fusion | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week04_sensor_technologies_fusion.ipynb) |
| | 5 | Deep Learning for Perception | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week05_deep_learning_perception.ipynb) |
| | 6 | Probabilistic State Estimation | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week06_probabilistic_state_estimation.ipynb) |
| | 7 | Localization and Mapping | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week07_localization_mapping.ipynb) |
| **Module III: Planning & Decision Making** | 8 | Mission & Behavior Planning | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week08_mission_behavior_planning.ipynb) |
| | 9 | Local Motion Planning | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week09_local_motion_planning.ipynb) |
| | 10 | Trajectory Optimization | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week10_trajectory_optimization.ipynb) |
| | 11 | Prediction and Interaction | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week11_prediction_interaction.ipynb) |
| **Module IV: Safety, Testing, & The Future** | 12 | Functional Safety & Redundancy | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week12_functional_safety_redundancy.ipynb) |
| | 13 | Testing, Validation, & Ethics | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week13_testing_validation_ethics.ipynb) |
| | 14 | Connected & Cooperative Driving | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week14_connected_cooperative_driving.ipynb) |
| | 15 | Final Project Presentation | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/week15_final_project_presentation.ipynb) |

### Documentation
- **`CLAUDE.md`** - Technical documentation for developers and Claude Code

## 🚀 Quick Start

### Option 1: Browser (No Installation)

Simply open the HTML files directly in any modern web browser:

```bash
# Windows
start kalman_ball_chase.html

# macOS
open kalman_ball_chase.html

# Linux
xdg-open kalman_ball_chase.html
```

No build process, no dependencies, no server required!

### Option 2: Jupyter Notebooks (Local)

Requires Python with Jupyter:

```bash
# Install dependencies
pip install numpy matplotlib ipywidgets jupyter

# Launch notebook
jupyter notebook kalman_ball_chase.ipynb
```

Then execute all cells to see the interactive animation.

### Option 3: Google Colab (Cloud-Based, No Installation)

Click the "Open in Colab" badges above to run the notebooks directly in your browser with zero setup!

## 🧮 Algorithms Explained

### Kalman Filter
**Purpose:** Optimal state estimation from noisy measurements

The Kalman filter operates in two phases:
1. **PREDICT:** Use physics model to predict future state
2. **UPDATE:** Correct prediction using noisy measurements

**Key Features:**
- Estimates both position AND velocity from position-only measurements
- Automatically balances trust between predictions and measurements
- Mathematically optimal under Gaussian noise assumptions
- Smooths out measurement noise

**Mathematical Foundation:**
```
State vector: [x, y, vx, vy]  (4D)
Measurement: [x_noisy, y_noisy]  (2D)

Prediction: x̂⁻(k) = F·x̂(k-1)
           P⁻(k) = F·P(k-1)·Fᵀ + Q

Update: K(k) = P⁻(k)·Hᵀ·[H·P⁻(k)·Hᵀ + R]⁻¹  (Kalman Gain)
        x̂(k) = x̂⁻(k) + K(k)·[z(k) - H·x̂⁻(k)]
        P(k) = (I - K(k)·H)·P⁻(k)
```

### PID Controller
**Purpose:** Minimize tracking error through feedback control

The PID controller combines three feedback terms:
1. **P (Proportional):** React to current error
2. **I (Integral):** Eliminate accumulated error over time
3. **D (Derivative):** Dampen oscillations by responding to error rate

**Key Features:**
- Simple three-parameter tuning (Kp, Ki, Kd)
- No system model required
- Robust to disturbances
- 70+ years of proven industrial use

**Mathematical Foundation:**
```
Control Law: u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt

Where:
- e(t) = target - current (error)
- Kp = proportional gain (immediate response)
- Ki = integral gain (eliminates steady-state error)
- Kd = derivative gain (reduces overshoot)
```

## 🎮 Interactive Controls

### HTML Versions
- **Reset** - Restart simulation from initial positions
- **Pause/Resume** - Freeze/unfreeze animation
- **Measurement Noise Slider** (Kalman only) - Adjust sensor noise (1-150)
- **Circle Radius Slider** - Change target's circular path radius (50-200 pixels)

### Jupyter Versions
Same controls implemented using `ipywidgets` for interactive manipulation within notebooks.

## 📊 Visualization Features

Both implementations display:
- **Blue Ball** - Target moving in circular path
- **Red Ball** - Hunter attempting to catch the target
- **Circular Path** - Dashed white circle showing target trajectory
- **Motion Trails** - Colored trails showing recent movement history
- **Prediction Indicator** (Kalman only) - Green dot showing predicted future position
- **Catch Indicator** - Orange highlight when hunter catches target

## 🧪 Experiment Ideas

### Kalman Filter
1. **Increase measurement noise** - Watch how the filter smooths noisy data
2. **Change radius** - See how prediction adapts to different motion scales
3. **Observe the green prediction dot** - Notice how it leads the blue ball

### PID Controller
1. **Adjust radius** - See how the predictive steering adapts
2. **Watch convergence** - Notice how the red ball intercepts the path
3. **Compare with Kalman** - Run both side-by-side to see differences

## 📚 Educational Value

This project demonstrates:

### Control Theory Concepts
- State estimation vs. feedback control
- Prediction and correction cycles
- Noise handling and filtering
- Real-time control algorithms

### Mathematical Foundations
- Linear algebra (matrix operations)
- Probability theory (Gaussian distributions)
- Calculus (derivatives and integrals)
- Optimization (minimizing error)

### Practical Applications
- Autonomous vehicle path planning
- Sensor fusion in robotics
- Target tracking in defense systems
- Industrial process control

## 🔍 Technical Details

### Ball Physics
- **Target (Blue):** Moves at constant angular velocity (0.008 rad/frame)
- **Hunter (Red):** Matches target's linear speed (~0.96 pixels/frame)
- **Frame Rate:** ~60 FPS (0.016 seconds per frame)
- **Catch Distance:** 25 pixels

### Kalman Filter Parameters
- **Process Noise Q:** Controls trust in motion model
- **Measurement Noise R:** Controls trust in sensor readings
- **State Dimension:** 4 (x, y, vx, vy)
- **Measurement Dimension:** 2 (x, y)

### PID Controller Parameters
- **Kp:** 0.1 (proportional gain)
- **Ki:** 0.02 (integral gain)
- **Kd:** 0.15 (derivative gain)
- **Anti-windup:** Integral clamped to ±1000

## 🎓 When to Use Each Algorithm

### Use Kalman Filter When:
- ✅ Measurements are very noisy
- ✅ You need to estimate hidden states (like velocity from position)
- ✅ System dynamics are well-understood and linear
- ✅ You want mathematical optimality guarantees
- ✅ Computational resources are available (O(n³) complexity)

### Use PID Controller When:
- ✅ You need simple, fast control
- ✅ System is well-behaved and easily controllable
- ✅ Measurements are relatively clean
- ✅ Computational constraints exist (O(1) complexity)
- ✅ System dynamics are unknown or hard to model

### Use Both Together:
- 🎯 Kalman filter estimates true state from noisy sensors
- 🎯 PID controller generates control commands based on estimates
- 🎯 Best of both worlds: optimal estimation + robust control

## 🛠️ Technology Stack

### HTML Versions
- Pure JavaScript (ES6+)
- HTML5 Canvas API for graphics
- MathJax 3.2.2 for LaTeX rendering
- No external dependencies (runs offline)

### Jupyter Versions
- Python 3.8+
- NumPy (matrix operations)
- Matplotlib (visualization and animation)
- ipywidgets (interactive controls)
- Jupyter Notebook

## 🎓 Course Syllabus: Autonomous Vehicle Systems Engineering

This project serves as supplementary material for the following advanced course curriculum:

### Course Overview

| **Aspect** | **Detail** |
|------------|------------|
| **Course Name** | Autonomous Vehicle Systems Engineering: From Sensor to Control |
| **Course Number** | AVSE 501 / CS 4XX |
| **Credits** | 3.0 |
| **Prerequisites** | Proficiency in Python (and/or C++), Linear Algebra, Calculus, and Basic Probability/Statistics. Prior exposure to basic Control Systems or Machine Learning is recommended. |
| **Primary Tools** | ROS (Robot Operating System), Python, C++, CARLA/Gazebo Simulation Environment, NumPy. |

### 🎯 Course Goals

- Understand the full software and hardware stack of a modern Level 4 Autonomous Vehicle (AV)
- Master the core algorithms for Perception, Localization, Planning, and Control
- Develop and implement key AV algorithms in a realistic simulation environment
- Analyze and address the critical challenges of functional safety and reliability

### 📚 Weekly Course Schedule

#### **Module I: Introduction & Foundations (Weeks 1-3)**

| Week | Topic | Key Concepts & Deliverables |
|------|-------|----------------------------|
| **1** | Introduction to Autonomy | SAE J3016 Levels of Driving Automation; History and Landscape of AVs; The Sense-Plan-Act Paradigm; System Architecture (hardware/software). |
| **2** | Vehicle Dynamics & Kinematics | Vehicle Modeling (Kinematic Bicycle Model, Dynamic Bicycle Model); Coordinate Frames and Transformations; Ackermann Steering Geometry. |
| **3** | Control Theory & Actuation | Drive-by-Wire systems; Longitudinal Control (PID, Cruise Control); Lateral Control (Pure Pursuit, Stanley Method). |

#### **Module II: Perception & Localization (Weeks 4-7)**

| Week | Topic | Key Concepts & Deliverables |
|------|-------|----------------------------|
| **4** | Sensor Technologies & Fusion | Camera (Image Processing, Homography); LiDAR (Point Clouds, Range Data); Radar (Doppler effects); Sensor Synchronization and Calibration. |
| **5** | Deep Learning for Perception | Object Detection (YOLO, R-CNN); Semantic Segmentation (identifying road, lane lines); Tracking and Prediction (Multi-Object Tracking - MOT). |
| **6** | Probabilistic State Estimation | Modeling Uncertainty; Bayes Filter fundamentals; Introduction to Kalman Filters (Linear, Extended, Unscented) for tracking. |
| **7** | Localization and Mapping | Global Navigation Satellite Systems (GNSS/GPS); Simultaneous Localization and Mapping (SLAM); Particle Filters for Global Localization; High-Definition (HD) Map representation. |

#### **Module III: Planning & Decision Making (Weeks 8-11)**

| Week | Topic | Key Concepts & Deliverables |
|------|-------|----------------------------|
| **8** | Mission & Behavior Planning | Route Planning (Dijkstra, A* search on road network graphs); Finite State Machines (FSM) for Decision Making (e.g., lane change, stop, wait). |
| **9** | Local Motion Planning | Sampling-based Planners (RRT/RRT*); Search-based Planners (State Lattice); Generating a collision-free path. |
| **10** | Trajectory Optimization | Cost Functions (smoothness, speed, safety); Model Predictive Control (MPC) for planning and control integration; Real-time constraints. |
| **11** | Prediction and Interaction | Modeling driver behavior; Predicting the trajectories of other agents; Game Theory and Reinforcement Learning for complex intersections. |

#### **Module IV: Safety, Testing, & The Future (Weeks 12-15)**

| Week | Topic | Key Concepts & Deliverables |
|------|-------|----------------------------|
| **12** | Functional Safety & Redundancy | ISO 26262 framework; Redundant systems (steering, braking, compute); Fail-Operational vs. Fail-Safe design; Safety of the Intended Functionality (SOTIF). |
| **13** | Testing, Validation, & Ethics | Hardware-in-the-Loop (HIL) and Software-in-the-Loop (SIL) simulation; Edge Cases and Scenario Testing; AV Ethics (Liability, The Trolley Problem). |
| **14** | Connected & Cooperative Driving | Vehicle-to-Everything (V2X) Communication (V2V, V2I); Utilizing cloud computing; Over-the-Air (OTA) updates. |
| **15** | Final Project Presentation | Student presentations and live demonstrations of final projects. |

### 🔗 How This Project Fits

This repository specifically covers:
- **Week 3:** Control Theory & Actuation (PID Controller implementation)
- **Week 6:** Probabilistic State Estimation (Kalman Filter implementation)

The interactive simulations and Jupyter notebooks provide hands-on experience with these fundamental algorithms before applying them to full-scale autonomous vehicle systems.

---

## 📖 Further Reading

### Kalman Filter
- Rudolf E. Kalman, "A New Approach to Linear Filtering and Prediction Problems" (1960)
- [Wikipedia: Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter)
- Applications: GPS, spacecraft navigation, signal processing

### PID Controller
- [Wikipedia: PID Controller](https://en.wikipedia.org/wiki/PID_controller)
- Ziegler-Nichols tuning method
- Applications: Thermostats, cruise control, manufacturing

## 🤝 Contributing

This is an educational project. Feel free to:
- Experiment with parameters
- Add new visualization features
- Implement variants (Extended Kalman Filter, Adaptive PID, etc.)
- Create additional examples

## 📄 License

Open source educational project. Use freely for learning and teaching purposes.

## 🙏 Acknowledgments

Built to demonstrate fundamental control theory concepts through interactive visualization. Mathematical implementations follow standard textbook algorithms used in robotics, aerospace, and autonomous systems.

---

**Happy Learning! 🚀** Experiment with the controls and watch how these algorithms power real-world autonomous systems!

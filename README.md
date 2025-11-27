# Self-Driving Car Course: Autonomous Vehicle Systems Engineering

A comprehensive 15-week course on autonomous vehicle systems, from fundamentals to advanced topics. This repository contains interactive Jupyter notebooks, demonstrations, and complete implementations of key algorithms.

## 🎯 Overview

This educational project provides hands-on learning for autonomous vehicle technologies, including:
- Self-driving vehicle algorithms and architectures
- Control systems (PID, MPC, Pure Pursuit, Stanley)
- State estimation (Kalman Filter, Particle Filter)
- Path planning and trajectory optimization
- Sensor fusion and perception
- Functional safety and testing

## 📁 Project Structure

### Interactive Demonstrations

#### HTML Visualizations (Browser-Based, No Installation Required)
- **`kalman_ball_chase.html`** - Kalman Filter implementation with predictive state estimation
- **`pid_ball_chase.html`** - PID Controller implementation with feedback control

Simply open these files in any modern web browser to see the algorithms in action!

#### Core Jupyter Notebooks
- **`kalman_ball_chase.ipynb`** - Interactive notebook with Kalman Filter animation
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/kalman_ball_chase.ipynb)

- **`pid_ball_chase.ipynb`** - Interactive notebook with PID Controller animation
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/adiel2012/self-driving-car/blob/main/pid_ball_chase.ipynb)

### 15-Week Course Curriculum

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

## 🚀 Quick Start

### Option 1: Browser Demonstrations (No Installation)

Open the HTML files directly in any modern web browser:

```bash
# Windows
start kalman_ball_chase.html

# macOS
open kalman_ball_chase.html

# Linux
xdg-open kalman_ball_chase.html
```

### Option 2: Jupyter Notebooks (Local)

Requires Python 3.8+ with Jupyter:

```bash
# Install dependencies
pip install numpy matplotlib ipywidgets jupyter scipy

# Launch notebook
jupyter notebook week01_introduction_to_autonomy.ipynb
```

### Option 3: Google Colab (Cloud-Based, Zero Setup)

Click any "Open in Colab" badge above to run notebooks directly in your browser!

**For Google Colab users:**
- See [COLAB_SETUP.md](COLAB_SETUP.md) for setup instructions and optimization tips
- See [MEMORY_TIPS.md](MEMORY_TIPS.md) for memory management strategies
- Notebooks are optimized for Colab's memory limits (12.7 GB RAM)

## 📚 Course Syllabus

### **Autonomous Vehicle Systems Engineering**

| **Aspect** | **Detail** |
|------------|------------|
| **Course Name** | Autonomous Vehicle Systems Engineering: From Sensor to Control |
| **Course Number** | AVSE 501 / CS 4XX |
| **Credits** | 3.0 |
| **Prerequisites** | Python proficiency, Linear Algebra, Calculus, Basic Probability/Statistics. Recommended: Control Systems or Machine Learning background. |
| **Tools** | Python, NumPy, Matplotlib, ROS (optional), CARLA/Gazebo (optional) |

---

### 🎯 Course Goals

By the end of this course, students will be able to:

1. **Understand** the complete software and hardware stack of modern Level 4 Autonomous Vehicles
2. **Implement** core algorithms for Perception, Localization, Planning, and Control
3. **Analyze** functional safety requirements and redundancy strategies
4. **Evaluate** autonomous vehicle systems through testing and validation
5. **Apply** learned concepts to build a comprehensive final project

---

### 📖 Weekly Breakdown

#### **Module I: Introduction & Foundations (Weeks 1-3)**

**Week 1: Introduction to Autonomy**
- SAE J3016 Levels of Driving Automation (L0-L5)
- History: DARPA Grand Challenge to modern deployments
- Sense-Plan-Act paradigm
- System architecture (sensors, compute, actuators)
- Industry landscape (Waymo, Cruise, Tesla, etc.)

**Week 2: Vehicle Dynamics & Kinematics**
- Kinematic bicycle model for motion planning
- Dynamic bicycle model with tire forces
- Coordinate frame transformations (2D/3D)
- Ackermann steering geometry
- Implementation: Vehicle trajectory simulation

**Week 3: Control Theory & Actuation**
- Drive-by-Wire systems (steer, throttle, brake)
- PID controllers for longitudinal control (cruise control)
- Lateral control: Pure Pursuit algorithm
- Lateral control: Stanley method (DARPA Grand Challenge)
- Adaptive Cruise Control (ACC) with gap control

---

#### **Module II: Perception & Localization (Weeks 4-7)**

**Week 4: Sensor Technologies & Fusion**
- Camera: Image processing, homography, calibration
- LiDAR: Point clouds, range data, registration
- Radar: Doppler velocity, weather robustness
- Sensor synchronization and extrinsic calibration
- Early/late fusion strategies

**Week 5: Deep Learning for Perception**
- Object detection: YOLO, Faster R-CNN, PointNet
- Semantic segmentation for drivable area
- Lane detection algorithms
- Multi-Object Tracking (MOT)
- Challenges: Occlusions, adverse weather

**Week 6: Probabilistic State Estimation**
- Modeling uncertainty (Gaussian distributions)
- Bayes filter fundamentals
- Kalman Filter (linear systems)
- Extended Kalman Filter (EKF) for nonlinear systems
- Unscented Kalman Filter (UKF)
- Implementation: Vehicle tracking with noisy GPS

**Week 7: Localization and Mapping**
- GPS/GNSS: Accuracy and limitations
- IMU integration for dead reckoning
- Particle Filter for global localization
- SLAM (Simultaneous Localization and Mapping)
- HD Maps: Representation and use
- Implementation: Monte Carlo Localization

---

#### **Module III: Planning & Decision Making (Weeks 8-11)**

**Week 8: Mission & Behavior Planning**
- Route planning: Dijkstra, A* on road graphs
- Finite State Machines (FSM) for decision-making
- Behavior trees for complex logic
- Lane change decision-making
- Intersection handling

**Week 9: Local Motion Planning**
- Configuration space and obstacles
- Sampling-based planners: RRT, RRT*
- Search-based planners: State lattice, hybrid A*
- Collision checking (bounding boxes, separating axis)
- Implementation: RRT path planning

**Week 10: Trajectory Optimization**
- Cost functions: Smoothness, speed, safety
- Polynomial trajectory generation
- Model Predictive Control (MPC)
- Convex optimization (CVXPY)
- Real-time constraints and approximations
- Implementation: MPC for path tracking

**Week 11: Prediction and Interaction**
- Trajectory prediction: Constant velocity, CTRV models
- Social LSTM for pedestrian prediction
- Game theory for intersection scenarios
- Reinforcement Learning (Q-Learning) for highway merging
- Multi-agent coordination

---

#### **Module IV: Safety, Testing, & The Future (Weeks 12-15)**

**Week 12: Functional Safety & Redundancy**
- ISO 26262 functional safety standard
- ASIL ratings (Automotive Safety Integrity Level)
- Hazard Analysis and Risk Assessment (HARA)
- Redundant systems: Dual sensors, fail-operational designs
- SOTIF (Safety of the Intended Functionality)
- Implementation: ASIL calculator, fault tree analysis

**Week 13: Testing, Validation, & Ethics**
- Verification vs. Validation
- X-in-the-Loop: MiL, SiL, HiL, ViL
- Scenario-based testing and edge cases
- Simulation environments (CARLA, LGSVL)
- AV Ethics: Trolley problem, liability, transparency
- Implementation: SIL testing framework

**Week 14: Connected & Cooperative Driving**
- V2X communication: V2V, V2I, V2P, V2N
- DSRC vs. C-V2X technology comparison
- Cooperative Adaptive Cruise Control (CACC)
- Platooning for fuel efficiency
- Cloud computing and edge computing
- Over-the-Air (OTA) software updates
- Cybersecurity: Attack vectors and defenses
- Implementation: V2X network simulator, CACC platoon

**Week 15: Final Project Presentation**
- Students present integrated AV projects
- Example projects: Autonomous parking, highway pilot, intersection navigation
- Live demonstrations and Q&A
- Peer evaluation

---

## 🧮 Key Algorithms Implemented

### State Estimation
- **Kalman Filter**: Optimal estimation for linear systems
- **Extended Kalman Filter (EKF)**: Nonlinear state estimation
- **Particle Filter**: Global localization with multimodal distributions

### Control
- **PID Controller**: Classic feedback control for speed/steering
- **Pure Pursuit**: Geometric path tracking
- **Stanley Method**: DARPA Grand Challenge winning controller
- **Model Predictive Control (MPC)**: Optimization-based trajectory tracking

### Planning
- **A* Search**: Optimal graph-based route planning
- **RRT/RRT***: Sampling-based motion planning
- **Trajectory Optimization**: Smooth, safe path generation

### Perception & Prediction
- **Multi-Object Tracking**: Kalman Filter + Hungarian algorithm
- **Trajectory Prediction**: Motion models (CV, CA, CTRV)
- **Q-Learning**: Reinforcement learning for decision-making

### Safety & Testing
- **ASIL Calculator**: ISO 26262 safety integrity assessment
- **HARA**: Hazard analysis methodology
- **Redundancy Analysis**: Monte Carlo reliability simulation

---

## 🎮 Interactive Features

All notebooks include:
- **Live visualizations** with matplotlib animations (20+ plots per notebook)
- **Interactive controls** using ipywidgets
- **Step-by-step explanations** with mathematical derivations
- **Practical exercises** with detailed solutions
- **Comprehensive references** (400+ academic papers and resources)
- **Performance analysis** with statistical comparisons
- **Parameter sensitivity studies** for algorithm tuning

### 🆕 Recent Enhancements
- ✅ **Exercise Solutions**: Complete solutions added for Week 6 (Kalman Filters) and Week 9 (RRT*)
- ✅ **Google Colab Optimization**: Memory-optimized code for stable execution (93% iteration reduction in Week 9)
- ✅ **Enhanced Visualizations**: Convergence plots, box plots, parameter sensitivity analysis
- ✅ **Advanced Implementations**: Extended Kalman Filter, Unscented Kalman Filter, Dynamic RRT with moving obstacles

---

## 🧪 Hands-On Projects

### Demonstrations
1. **Ball Chase Simulations**: Kalman Filter vs. PID control comparison
2. **Vehicle Trajectory Tracking**: Multiple control strategies
3. **CACC Platoon**: Cooperative driving with V2X communication
4. **Lane Keeping System**: Integrated perception-planning-control

### Exercises (60+ total across all weeks)
- PID tuning for cruise control
- Coordinate transformations for sensor fusion
- Path planning with RRT
- MPC trajectory optimization
- HARA safety analysis
- Scenario-based testing design

**Exercise Solutions Available:**
- ✅ **Week 6**: 1D Kalman Filter, KF/EKF/UKF comparison with comprehensive analysis
- ✅ **Week 9**: RRT* convergence analysis, dynamic obstacle planning with space-time RRT
- Additional solutions included at the end of relevant notebooks after exercises

---

## 📊 Visualization Examples

The course includes rich visualizations:
- Vehicle motion with kinematic bicycle model
- Kalman Filter prediction and update steps
- Path planning algorithms (A*, RRT)
- Control performance comparison (tracking error, steering smoothness)
- Safety metrics (ASIL determination, fault trees)
- Multi-agent scenarios (intersection navigation, platooning)

---

## 🛠️ Technology Stack

### Required
- **Python 3.8+**
- **NumPy**: Matrix operations and numerical computing
- **Matplotlib**: Visualization and animation
- **SciPy**: Optimization and scientific computing

### Optional (for advanced topics)
- **CVXPY**: Convex optimization for MPC
- **ROS**: Robot Operating System (for real hardware integration)
- **CARLA/Gazebo**: 3D simulation environments

### HTML Demonstrations
- Pure JavaScript (ES6+)
- HTML5 Canvas API
- MathJax 3.2 for LaTeX rendering
- Zero dependencies (runs offline)

---

## 📖 Learning Resources

### Textbooks (Referenced in Course)
1. **Probabilistic Robotics** by Thrun, Burgard & Fox - State estimation and SLAM
2. **Planning Algorithms** by LaValle - Comprehensive planning coverage
3. **Vehicle Dynamics and Control** by Rajamani - Vehicle modeling and control

### Papers (400+ citations across all notebooks)
- Kalman (1960): Original Kalman Filter paper
- Thrun et al. (2006): Stanley (DARPA Grand Challenge winner)
- Paden et al. (2016): Survey of AV motion planning and control
- ISO 26262 (2018): Functional safety standard

### Online Resources
- Udacity Self-Driving Car Nanodegree
- MIT 6.S094: Deep Learning for Self-Driving Cars
- Coursera: Self-Driving Cars Specialization (University of Toronto)
- Apollo Auto and Autoware open-source platforms

---

## 🎓 Prerequisites

**Required:**
- Python programming (functions, classes, NumPy)
- Linear algebra (vectors, matrices, transformations)
- Calculus (derivatives, integrals, optimization)
- Basic probability (Gaussian distributions, Bayes' theorem)

**Recommended:**
- Control systems (transfer functions, feedback loops)
- Machine learning (neural networks, training)
- Computer vision basics
- Robotics fundamentals

---

## 🤝 Contributing

This is an educational project. Contributions welcome:
- Add new algorithm implementations
- Improve visualizations
- Create additional exercises
- Fix bugs or typos
- Enhance documentation

---

## 📄 License

Open source educational project. Use freely for learning and teaching purposes.

---

## 🙏 Acknowledgments

Course content developed using standard autonomous vehicle algorithms from academic research and industry practice. Special thanks to the autonomous vehicle research community for open publications and datasets.

Key datasets used:
- KITTI Vision Benchmark Suite
- nuScenes (Waymo, Motional)
- Waymo Open Dataset

---

## 🚀 Getting Started

1. **Clone the repository**:
   ```bash
   git clone https://github.com/adiel2012/self-driving-car.git
   cd self-driving-car
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Start with Week 1**:
   ```bash
   jupyter notebook week01_introduction_to_autonomy.ipynb
   ```

4. **Or try the browser demos**:
   - Open `kalman_ball_chase.html` or `pid_ball_chase.html` directly

---

**Happy Learning! 🚗💨**

Master the algorithms that power modern autonomous vehicles through hands-on implementation and interactive exploration!

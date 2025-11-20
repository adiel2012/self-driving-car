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
- **`kalman_ball_chase.ipynb`** - Interactive notebook with Kalman Filter animation
- **`pid_ball_chase.ipynb`** - Interactive notebook with PID Controller animation

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

### Option 2: Jupyter Notebooks

Requires Python with Jupyter:

```bash
# Install dependencies
pip install numpy matplotlib ipywidgets jupyter

# Launch notebook
jupyter notebook kalman_ball_chase.ipynb
```

Then execute all cells to see the interactive animation.

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

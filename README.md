# Adaptive-Control-Stepper-Motor

## Project Overview
This repository contains the MATLAB simulation and comparative analysis of advanced control architectures applied to a discrete-time NEMA 23 Hybrid Stepper Motor. The project evaluates the efficacy of batch-learning versus real-time adaptive control in mitigating unmodeled dynamics, measurement noise, and discontinuous friction.

📄 **[Read the Full Technical Report Here](StepperMotorControl_Report.pdf)** *(Note: Click "View raw" or download the PDF to ensure the internal document links work correctly).*

Three specific control topologies are benchmarked against a 1 Hz sinusoidal reference trajectory:
1. **Baseline PID Control:** Establishes the uncompensated error and noise floor.
2. **Iterative Learning Control (ILC):** Implements a lifted-system matrix approach ($G^{-1}$) with a zero-phase Q-filter to achieve noise-level tracking over repetitive trials.
3. **Discrete-Time MRAC (DEMCS):** Implements an Enhanced Minimal Control Synthesis algorithm with robust and integral adaptive terms to violently reject real-time disturbances.

## Repository Structure
* `StepperMotor_Control.m`: The master script. Initializes the plant physics, executes all three control simulations, and generates the comparative performance plots.
* `ilc.m`: The Iterative Learning Control function. Constructs the lower-triangular Toeplitz matrix and executes the batch-learning update law over 30 trials.
* `mrac_demcs.m`: The Model Reference Adaptive Control function. Solves the discrete Lyapunov equation and updates the $L, L_R, L_I$, and $L_E$ gains dynamically at every time step.

## Prerequisites
* MATLAB (R2021a or newer recommended)
* Control System Toolbox

## Usage
1. Clone this repository to your local machine.
2. Ensure `StepperMotor_Control.m`, `ilc.m`, and `mrac_demcs.m` are in the same active directory.
3. Run `StepperMotor_Control.m`.
4. The script will prompt you in the Command Window to define the ILC parameters:
   * **Cutoff Frequency [Hz]:** (e.g., 50)
   * **Learning Rate:** (e.g., 0.3)
5. The simulation will execute and automatically generate the time-domain tracking, actuator demand, and RMS convergence figures.

## Key Conclusions
* **ILC** is highly recommended for repetitive manufacturing trajectories (e.g., gantry operations), as its non-causal anticipation entirely eliminates phase lag while maintaining a smooth, safe control effort.
* **MRAC** successfully forces the plant to track an ideal reference model without prior practice, making it superior for unpredictable environments (e.g., throttle valve disturbances). However, causality constraints introduce phase lag on continuous high-frequency trajectories, and its aggressive real-time adaptation is highly susceptible to sensor noise.

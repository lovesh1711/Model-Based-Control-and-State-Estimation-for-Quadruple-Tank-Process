# Model Predictive Control for Quadruple Tank Process

This repository contains the implementation of Model Predictive Control (MPC) to regulate the levels of a quadruple tank process. The project explores both constrained and unconstrained MPC scenarios using a linearized discrete state-space model, incorporating Kalman filter-based state estimation.

## Problem Statement

The goal is to implement MPC for various control scenarios to regulate the levels of four interconnected tanks (\(h_1, h_2, h_3, h_4\)) in a quadruple tank system. Key parameters and requirements are as follows:

- **Initial State Values**: \([h_1, h_2, h_3, h_4] = [12.4, 12.7, 1.8, 1.4]\)
- **Sampling Time (\(T_s\))**: \(0.1 \, s\)
- **Controller Gain (\(K_c\))**: \(1 V/cm)
- **The constraints are**:  
-  Delta U_{min} = 5*[-1, -1]^T
-  Delta U_{max} = 5*[1, 1]^T
-  U_{min} = 0*[-1, -1]^T  
-  U_{max} = 20*[1, 1]^T 



The system incorporates integrated white noise as state noise and measurement noise. A Kalman filter (from Project 1) is used for state estimation.

## Implementation Tasks

### A) Unconstrained MPC
1. **Scenario**: Control \(h_1, h_2\) with all states measured, aiming for the set-point \([h_1, h_2] = [13.4, 13.7]\).
2. **Steps**:
   - Use heuristics to choose prediction and control horizons.
   - Analyze closed-loop stability of the system at:
     - The initial move.
     - When the system stabilizes near the set-point.
   - Examine poles at both locations and provide comments.

### B) Constrained MPC
1. **Scenario**: Control \(h_3, h_4\) while measuring \(h_1, h_2\), with set-point \([h_3, h_4] = [2.8, 2.4]\).
2. **Steps**:
   - Implement MPC with constraints.
   - Evaluate the impact of Kalman filter performance on MPC by experimenting with Kalman gain parameters.

### C) Constrained MPC for Multiple Cases
1. **Case 1**: Control \(h_2, h_3\) while measuring \(h_1, h_4\), with set-point \([h_2, h_3] = [13.7, 2.8]\).
2. **Case 2**: Control \(h_1, h_3\) while measuring \(h_2, h_4\), with set-point \([h_1, h_3] = [13.7, 2.4]\).

## Key Objectives
- Analyze closed-loop system behavior for each scenario.
- Assess the impact of constraints and Kalman filter performance on MPC control.
- Validate system stability and controller performance for different configurations.

## Tools
- MATLAB: For simulation and implementation of MPC algorithms.

Feel free to explore the repository for the MATLAB code and simulation results!


# Model-Based-Control-and-State-Estimation-for-Quadruple-Tank-Process

## Problem Statement



Based on the paper, titled **"The Quadruple-Tank Process: A Multivariable Laboratory Process with an Adjustable Zero"** by Karl Henrik Johansson (attached) and the diagram given below, 
![image](https://github.com/user-attachments/assets/8427abbe-d2aa-474e-b1ac-ecff481807d5)


![image](https://github.com/user-attachments/assets/71506193-9469-4960-b6b9-f08d6e83e7e0)


where:

- The measurements available are for:
  - **Tank 1**: \( h_1 \)
  - **Tank 2**: \( h_2 \)

## Governing Equations
The governing equations are as shown below

![image](https://github.com/user-attachments/assets/aaf7c622-dfe5-4751-98d5-197980244460)


## Matrices A, B, & C
The matrices \( A \), \( B \), and \( C \) can be computed from the **Linearized State & Measurement equations** (around an operating point), as shown below
![image](https://github.com/user-attachments/assets/5af63468-35da-478b-9980-87f3be27efe8)

**below are the initial values to be assumed**

```matlab
% initialization of all the parameters of the four tank system
clc; clear; close all;

A1 = 28; %(cm^2)
A2 = 32;
A3 = 28;
A4 = 32;

a1 = 0.071; a3 = 0.071; %(cm^2)
a2 = 0.057; a4 = 0.057;

kc = 0.5; % (V/cm)
g = 981; %(cm/s^2)

gamma1 = 0.7; gamma2 = 0.6; % constants, determined from valve position
k1 = 3.33; k2 = 3.35; %[cm^3/Vs]
kc = 0.5; % [V/cm]

v1 = 3; v2 = 3; % (V)
h0 = [12.4; 12.7; 1.8; 1.4];

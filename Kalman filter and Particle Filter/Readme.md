# Problem Statement

## Data Generation for Part-A & B

### Q-1
Solve the set of non-linear equations of the four-tank system, using the forward difference method or by using ODE-45 of Matlab (or equivalent in Python, etc.), for generating a typical data set of 10,000. This data set is to be used for comparing the performance of the Kalman Filter & Particle Filter.

---

## Part-A: Kalman Filter

### Q-2:
Formulate a **Kalman Filter** for this four-tank problem, for estimating **h-3 & h-4** (not measured) and obtaining the filtered values for **h-1 & h-2** (measured). Verify the resulting values of **h-1, h-2, h-3 & h-4**, by comparing the same with the generated data set.

### Q-3:
Obtain the below plots to understand the performance of Kalman filter
- **Prior Estimate of** x(k-) **(state variable), with** k
- **Posterior Estimate of** x(k-) **(state variable), with** k
- **Prior Estimate of Covariance,** P(k-) **with** k
- **Posterior Estimate of Covariance,** P(k+) **with** k
- **Kalman-Gain,** K(k) **with** k
- **Prior Residues/Innovations,** r(k-) **with** k
- **Posterior Residues,** r(k+) **with** k



---

## Part-B: Particle Filter

### Q-4:
Formulate a **Particle Filter** for the above four-tank problem for estimating **h-3 & h-4** (not measured) and obtaining the filtered values for **h-1 & h-2** (measured). Verify the resulting estimate by comparing the same with the generated data set (from Q-1).


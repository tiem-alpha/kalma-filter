
**KALMAN FILTER**

Date: 2026-06-01
Context: Embedded Systems / Signal Processing Course


# SECTION 1: MATHEMATICAL FOUNDATIONS


1.1 PROBABILITY & STATISTICS REVIEW
-------------------------------------
The Kalman filter is fundamentally a Bayesian estimator. It uses probability
theory as a framework for updating beliefs about system state as new data
arrives. Understanding the following concepts is essential:

  **GAUSSIAN (NORMAL) DISTRIBUTION**
  - Defined by mean (mu) and variance (sigma^2)
  - A scalar Gaussian: p(x) = (1/sqrt(2*pi*sigma^2)) * exp(-(x-mu)^2 / 2*sigma^2)
  - The Kalman filter assumes all noise is Gaussian — this is what makes it
    the optimal (minimum mean square error) linear estimator.
  - In multiple dimensions: the multivariate Gaussian is parameterized by
    mean vector mu and covariance matrix P.

  **MEAN, VARIANCE, COVARIANCE**
  - Mean: expected value of a random variable, E[x]
  - Variance: E[(x - mu)^2] — measures spread of a single variable
  - Covariance: E[(x - mu_x)(y - mu_y)] — measures how two variables vary together
  - Covariance matrix P: symmetric, positive semi-definite matrix
    P[i][j] = cov(x_i, x_j); diagonal entries are individual variances

  **BAYES' THEOREM**
  - p(x | z) = p(z | x) * p(x) / p(z)
    where:
      p(x)     = prior (what we believed before measurement)
      p(z | x) = likelihood (how probable is measurement z given state x)
      p(x | z) = posterior (updated belief after seeing measurement z)
  - The Kalman filter is a recursive application of Bayes' theorem under
    the assumption of Gaussian distributions and linear dynamics.

  **CONDITIONAL PROBABILITY**
  - p(A | B) = p(A ∩ B) / p(B)
  - Central to prediction: we want p(x_k | z_{1:k}), the probability of
    the current state given all measurements up to now.

1.2 LINEAR ALGEBRA
------------------
The Kalman filter operates entirely through matrix equations. The following
linear algebra operations are required:

  **KEY MATRICES IN THE FILTER**
  - State vector x: column vector of system states (e.g., position, velocity)
  - State transition matrix A (or F): maps state at k-1 to predicted state at k
  - Control input matrix B: maps control input u to state change
  - Observation matrix H: maps state space to measurement space
  - Process noise covariance Q: uncertainty in the system model
  - Measurement noise covariance R: uncertainty in the sensor readings
  - Error covariance matrix P: current estimate uncertainty

  **REQUIRED OPERATIONS**
  - Matrix multiplication: C = A * B
  - Matrix transpose: A^T — rows become columns
  - Matrix inverse: A^-1, required for the Kalman gain computation
    (Note: for numerical stability on embedded systems, avoid explicit
    inversion — use Cholesky decomposition instead)
  - Identity matrix I: I * A = A
  - Determinant: used to assess whether a matrix is invertible
  - Positive definiteness: P must always remain symmetric positive definite;
    a violation indicates numerical instability

1.3 STATE-SPACE REPRESENTATION
-------------------------------
State-space models describe a system as a set of first-order differential
(or difference) equations. They are the mathematical framework underlying
the Kalman filter.

  **DISCRETE-TIME STATE EQUATIONS**

    Process model (system dynamics):
      x_k = A * x_{k-1} + B * u_k + w_k

      where:
        x_k     = state vector at time step k
        A       = state transition matrix (n x n)
        x_{k-1} = previous state vector
        B       = control input matrix (n x m)
        u_k     = control input vector (m x 1)
        w_k     = process noise ~ N(0, Q)

    Observation model (measurement):
      z_k = H * x_k + v_k

      where:
        z_k = measurement vector at time k
        H   = observation matrix (p x n)
        v_k = measurement noise ~ N(0, R)

  **NOISE ASSUMPTIONS**
  - Process noise w_k: zero-mean Gaussian, covariance matrix Q
    Models unmodeled dynamics, disturbances, model imperfections
  - Measurement noise v_k: zero-mean Gaussian, covariance matrix R
    Models sensor inaccuracy, quantization error, environmental interference
  - w_k and v_k are assumed to be uncorrelated with each other and over time

  EXAMPLE: 1D POSITION-VELOCITY TRACKING
    State:  x = [position, velocity]^T
    A = [[1, dt],   (position += velocity * dt)
         [0,  1]]   (velocity stays constant, absent control)
    H = [1, 0]      (we only measure position, not velocity)
    Q = small matrix (trust the model)
    R = scalar (sensor noise variance)



# SECTION 2: KALMAN FILTER THEORY


2.1 CORE CONCEPT
----------------
The Kalman filter, invented by Rudolf E. Kalman in 1960, is a recursive
algorithm that produces the optimal linear estimate of an unknown system state
from noisy measurements. It is optimal in the sense that it minimizes the
mean square error (MSE) of the estimated state — provided the system is linear
and the noise is Gaussian.

  KEY PROPERTIES
  - Recursive: only the previous estimate and the new measurement are needed;
    no need to store the entire history of measurements
  - Optimal: under Gaussian, linear assumptions it achieves the Cramér-Rao
    lower bound on estimation error
  - Two-phase cycle: PREDICT (extrapolate state forward) + UPDATE (correct
    with new measurement)
  - The filter continuously balances trust in the model vs. trust in the sensor,
    governed by the Kalman gain K

2.2 PREDICTION STEP (Time Update)
-----------------------------------
The prediction step projects the state estimate and its uncertainty forward
in time, before a new measurement is received.

  STATE PREDICTION (State Extrapolation Equation):
    x_hat_{k|k-1} = A * x_hat_{k-1|k-1} + B * u_k

    - x_hat_{k|k-1}: predicted (a priori) state estimate at step k
    - Uses current best estimate and the system dynamics model
    - Also called: predictor equation, transition equation

  COVARIANCE PREDICTION (Covariance Extrapolation Equation):
    P_{k|k-1} = A * P_{k-1|k-1} * A^T + Q

    - P_{k|k-1}: predicted (a priori) error covariance
    - A * P * A^T: propagates existing uncertainty through dynamics
    - + Q: adds process noise uncertainty accumulated since last step
    - Uncertainty always grows during prediction (Q > 0)

2.3 UPDATE STEP (Measurement Update / Correction)
---------------------------------------------------
When a new measurement z_k arrives, the update step corrects the prediction.

  INNOVATION (measurement residual):
    y_k = z_k - H * x_hat_{k|k-1}
    - Difference between actual measurement and predicted measurement
    - If the filter is working well, this should be white (zero-mean, uncorrelated)

  INNOVATION COVARIANCE:
    S_k = H * P_{k|k-1} * H^T + R

  KALMAN GAIN (Weight Equation):
    K_k = P_{k|k-1} * H^T * (H * P_{k|k-1} * H^T + R)^{-1}
        = P_{k|k-1} * H^T * S_k^{-1}

    - K_k balances model uncertainty (P) vs. sensor uncertainty (R)
    - If R is large (noisy sensor):  K → small → trust model more
    - If P is large (uncertain model): K → large → trust sensor more
    - Dimensions: (n x p), where n = states, p = measurements

  STATE UPDATE (Filtering Equation):
    x_hat_{k|k} = x_hat_{k|k-1} + K_k * (z_k - H * x_hat_{k|k-1})
                = x_hat_{k|k-1} + K_k * y_k

    - Corrects prediction by weighting the innovation with Kalman gain

  COVARIANCE UPDATE (Corrector Equation):
    Standard form:
      P_{k|k} = (I - K_k * H) * P_{k|k-1}

    Joseph form (numerically stable — PREFERRED for embedded use):
      P_{k|k} = (I - K_k * H) * P_{k|k-1} * (I - K_k * H)^T + K_k * R * K_k^T

    - After update, uncertainty P always decreases (measurement adds information)
    - Joseph form guarantees symmetry and positive-definiteness of P, even with
      floating-point rounding errors

2.4 TUNING PARAMETERS
-----------------------
The filter's behavior is almost entirely governed by Q and R:

  EFFECT OF Q (Process Noise Covariance)
  - Large Q: filter trusts the model less, reacts faster to measurements,
    more responsive but noisier output
  - Small Q: filter trusts the model more, smoother output,
    slower to respond to real changes

  EFFECT OF R (Measurement Noise Covariance)
  - Large R: sensor is noisy, filter relies more on prediction
  - Small R: sensor is accurate, filter tracks measurements closely

  Q/R RATIO IS WHAT MATTERS
  - Only the ratio Q/R affects behavior, not absolute magnitudes
  - Typical starting point: set R from sensor datasheet noise specs;
    tune Q experimentally

  INITIAL CONDITIONS
  - x_hat_0: initial state estimate (set to first measurement or zero)
  - P_0: initial covariance (set large if uncertain about initial state)

  CONVERGENCE
  - The Kalman gain K_k converges to a steady-state value K_inf when
    A, Q, R, H are time-invariant (LTI system)
  - Steady-state gain can be precomputed offline for embedded systems
    to save computation at runtime



# SECTION 3: EXTENDED & VARIANTS


3.1 EXTENDED KALMAN FILTER (EKF)
----------------------------------
The standard Kalman filter only handles linear systems. Most real-world
systems (IMU attitude, robot kinematics, ballistics) are nonlinear.
The EKF handles nonlinearity by linearizing around the current estimate.

  NONLINEAR SYSTEM MODEL
    Process:     x_k = f(x_{k-1}, u_k) + w_k
    Measurement: z_k = h(x_k) + v_k

    where f() and h() are nonlinear functions.

  LINEARIZATION VIA JACOBIANS
  - The EKF approximates f and h with their first-order Taylor expansion
    at the current estimate
  - Jacobian of f: F_k = df/dx |_{x_hat_{k-1|k-1}}   (state transition Jacobian)
  - Jacobian of h: H_k = dh/dx |_{x_hat_{k|k-1}}      (observation Jacobian)
  - These Jacobians replace the constant A and H matrices in standard KF

  EKF EQUATIONS
    Predict:
      x_hat_{k|k-1} = f(x_hat_{k-1|k-1}, u_k)
      P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + Q

    Update:
      K_k = P_{k|k-1} * H_k^T * (H_k * P_{k|k-1} * H_k^T + R)^{-1}
      x_hat_{k|k} = x_hat_{k|k-1} + K_k * (z_k - h(x_hat_{k|k-1}))
      P_{k|k} = (I - K_k * H_k) * P_{k|k-1}

  LIMITATIONS OF EKF
  - Linearization introduces errors if the system is highly nonlinear
  - Can diverge if the initial estimate is far from the true state
  - Jacobian computation is analytically complex and error-prone
  - Not guaranteed optimal — only approximately optimal

  COMMON APPLICATIONS
  - IMU attitude estimation (quaternion-based)
  - GPS/IMU dead-reckoning navigation
  - Robot localization (SLAM)
  - Missile/aircraft tracking

3.2 UNSCENTED KALMAN FILTER (UKF)
-----------------------------------
The UKF is an alternative to EKF for nonlinear systems that avoids
Jacobian computation and achieves at least second-order accuracy.

  **CORE IDEA: SIGMA POINTS**
  - Instead of linearizing, the UKF propagates a small, deterministic set
    of carefully chosen sample points (sigma points) through the true
    nonlinear function
  - The sigma points capture the mean and covariance of the prior distribution

  **SIGMA POINT SELECTION** (for n-dimensional state)
  - 2n + 1 sigma points are computed from the current mean and covariance
  - Propagated through f() and h() without any approximation
  - Weighted mean and covariance are computed from propagated points

 **UKF vs EKF COMPARISON**

| Property              | EKF                       | UKF                       |
| :-------------------- | :------------------------ | :------------------------ |
| Nonlinearity handling | First-order linearization | Sigma point propagation   |
| Accuracy              | First order               | Second order (or higher)  |
| Jacobians required    | Yes (manual derivation)   | No                        |
| Computation cost      | Lower                     | Higher (2n+1 evaluations) |
| Divergence risk       | Higher                    | Lower                     |
| Best for              | Mildly nonlinear          | Highly nonlinear          |

3.3 COMPLEMENTARY FILTER
--------------------------
A simpler, computationally cheap alternative for attitude estimation,
commonly used on resource-constrained embedded systems.

  CONCEPT
  - Exploits the complementary frequency characteristics of sensors:
    * Gyroscope: accurate short-term (low noise), but drifts long-term
    * Accelerometer: noisy short-term, but stable long-term reference

  EQUATION
    angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle

    where alpha (typically 0.98) is the complementary filter coefficient.
    alpha close to 1 → trust gyro more; alpha close to 0 → trust accel more.

  ADVANTAGES
  - Extremely low CPU and memory cost
  - Easy to implement in C (no matrix math)
  - Good enough for many drone/robot attitude applications

  DISADVANTAGES
  - Not statistically optimal (no principled noise model)
  - Fixed tuning — cannot adapt to varying sensor noise
  - Cannot estimate biases or fuse more than two sensors cleanly



# SECTION 4: IMPLEMENTATION (EMBEDDED SYSTEMS)


4.1 ALGORITHM IN C/C++ FOR EMBEDDED SYSTEMS
---------------------------------------------
Implementing a Kalman filter on a microcontroller requires careful attention
to resource constraints (RAM, CPU cycles, no OS) and numerical precision.

  TYPICAL IMPLEMENTATION STRUCTURE (C pseudocode)

    // State and covariance
    float x[N];         // state vector (N states)
    float P[N][N];      // error covariance matrix
    float Q[N][N];      // process noise covariance
    float R[M][M];      // measurement noise covariance (M measurements)
    float A[N][N];      // state transition matrix
    float H[M][N];      // observation matrix

    void kalman_predict() {
        x = A * x + B * u;          // state prediction
        P = A * P * A_T + Q;        // covariance prediction
    }

    void kalman_update(float z[M]) {
        S = H * P * H_T + R;        // innovation covariance
        K = P * H_T * inv(S);       // Kalman gain
        y = z - H * x;              // innovation
        x = x + K * y;              // state update
        P = (I - K*H) * P;          // covariance update (or Joseph form)
    }

  MATRIX LIBRARY OPTIONS FOR EMBEDDED C
  - CMSIS-DSP (ARM): arm_mat_mult_f32, arm_mat_inverse_f32, etc.
    Optimized for ARM Cortex-M processors, uses SIMD instructions on M4/M7
  - Eigen (C++): full-featured, header-only, but large code size
  - Custom minimal matrix library: write only the ops you need
    (multiply, transpose, add, inverse for small matrices)
  - libfixkalman: for processors without FPU (Cortex-M0/M3),
    uses 16.16 fixed-point arithmetic via libfixmath

  FIXED-POINT vs FLOATING-POINT
  +------------------+----------------------------+---------------------------+
  | Property         | Fixed-Point (Q format)     | Floating-Point (float)    |
  +------------------+----------------------------+---------------------------+
  | Target hardware  | Cortex-M0/M3, no FPU       | Cortex-M4/M7 with FPU     |
  | Precision        | Limited, overflow risk     | 32-bit (6-7 decimal digits)|
  | Speed            | Fast on non-FPU MCUs       | Fast on FPU MCUs          |
  | Complexity       | High (manual scaling)      | Low                       |
  | Recommended      | Severely constrained MCUs  | Most modern MCUs          |
  +------------------+----------------------------+---------------------------+
  - NOTE: single-precision float (32-bit) can lose accuracy in iterative
    matrix updates; double (64-bit) is safer for high-dimension systems

  MEMORY FOOTPRINT
  - For a state vector of size N, covariance P requires N^2 floats
  - Example: 9-state (full IMU) filter → P = 9x9 = 81 floats = 324 bytes
  - Symmetric P: can store only upper triangle → N*(N+1)/2 elements
  - Total working matrices: ~5-6 matrices → for N=6: ~6 * 36 * 4 = 864 bytes

4.2 NUMERICAL STABILITY
-------------------------
On embedded targets with limited floating-point precision, naive
implementations of the Kalman filter can diverge due to rounding errors.

  PROBLEM: P LOSES POSITIVE DEFINITENESS
  - The standard covariance update P = (I - KH) * P can, over time,
    make P non-symmetric or non-positive-definite due to floating-point errors
  - A non-positive-definite P leads to negative variances → filter divergence

  SOLUTION 1: JOSEPH FORM
  - Numerically stable form of the covariance update:
    P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
  - Guarantees symmetry even in the presence of floating-point errors
  - Higher computation cost (more matrix multiplications), but essential
    for reliable embedded deployment

  SOLUTION 2: CHOLESKY DECOMPOSITION (Square-Root Filter)
  - Represent P as P = S * S^T (Cholesky factorization, S is lower triangular)
  - Propagate S instead of P through the filter equations
  - The condition number of S is sqrt(cond(P)) → numerically much better
  - Prevents P from losing positive-definiteness by construction
  - Used in high-reliability applications (aerospace, automotive ECU)
  - CMSIS-DSP provides: arm_mat_cholesky_f32()

  SOLUTION 3: SYMMETRY ENFORCEMENT
  - After each update, force: P = (P + P^T) / 2
  - Simple but only partially corrects errors; use alongside Joseph form

4.3 REAL-TIME CONSTRAINTS
--------------------------
A Kalman filter in embedded must complete its predict-update cycle within
the sensor sampling period (e.g., < 1 ms for a 1 kHz IMU).

  EXECUTION TIME OPTIMIZATION
  - Precompute constant matrices (A, H, Q, R) offline, store in flash
  - If system is LTI (time-invariant), precompute steady-state Kalman gain K_ss
    offline and hardcode it — eliminates all matrix operations at runtime
  - Use CMSIS-DSP intrinsics for ARM; they use SIMD instructions
  - Unroll small matrix loops (e.g., 3x3, 4x4) manually for cache efficiency
  - Avoid dynamic memory allocation (malloc) — use static arrays only

  INTERRUPT-DRIVEN vs POLLING
  - Interrupt-driven: sensor triggers ISR → calls kalman_update()
    Pro: precise timing; Con: ISR must be short, offload heavy math to main loop
  - Polling: main loop reads sensor at fixed rate → runs full filter
    Pro: simpler code; Con: timing jitter if loop has variable work

  RTOS INTEGRATION
  - Dedicate a high-priority task to the filter (e.g., FreeRTOS task)
  - Use a mutex or double-buffer to share state with other tasks
  - Set task period = sensor sampling period (e.g., 1 ms for IMU)
  - Monitor task worst-case execution time (WCET) with a logic analyzer or DWT cycle counter



# SECTION 5: SENSOR FUSION APPLICATIONS (EMBEDDED FOCUS)


5.1 IMU ATTITUDE ESTIMATION
-----------------------------
The most common embedded Kalman filter application: estimating roll, pitch,
and yaw from MEMS inertial sensors.

  SENSOR CHARACTERISTICS AND COMPLEMENTARY NATURE
  - Gyroscope (rate sensor):
    * Measures angular rate (deg/s or rad/s)
    * Accurate short-term, but integrates drift over time (gyro drift)
    * Drift caused by temperature, bias instability, vibration
  - Accelerometer:
    * Measures specific force (gravity + linear acceleration)
    * Accurate long-term gravity reference for roll and pitch
    * Noisy during motion (cannot separate gravity from linear acceleration)
  - Magnetometer (for yaw/heading):
    * Measures Earth's magnetic field for absolute yaw reference
    * Susceptible to hard/soft iron distortion from motors, PCB traces

  STATE VECTOR (6-DOF: accel + gyro)
    x = [roll, pitch, gyro_bias_x, gyro_bias_y]^T
    or use quaternion representation to avoid gimbal lock:
    x = [q0, q1, q2, q3, gyro_bias_x, gyro_bias_y, gyro_bias_z]^T

  PROCESS MODEL
  - State propagation uses gyroscope integration:
    q_{k} = q_{k-1} + 0.5 * Omega(omega_gyro - bias) * q_{k-1} * dt
  - Gyro bias modeled as random walk: bias_k = bias_{k-1} + noise

  MEASUREMENT MODEL
  - Accelerometer provides roll and pitch via:
    roll  = atan2(ay, az)
    pitch = atan2(-ax, sqrt(ay^2 + az^2))
  - EKF needed because this relationship is nonlinear

  REAL-WORLD IMPLEMENTATIONS
  - ArduPilot / Betaflight drone flight controllers: EKF-based 9-DOF fusion
  - Self-balancing robots: Kalman-filtered pitch angle fed to PID controller
  - VR headsets: low-latency Kalman fusion for sub-millisecond head tracking

5.2 POSITION / VELOCITY ESTIMATION
-------------------------------------
  GPS + IMU FUSION (Loose Coupling)
  - GPS provides absolute position + velocity (low rate, ~1-10 Hz, ~2-5 m accuracy)
  - IMU provides high-rate acceleration (100-1000 Hz), deadreckons between GPS fixes
  - Kalman state: [position, velocity, accel_bias]
  - During GPS outage: rely on IMU dead-reckoning (error grows with time)
  - EKF used for nonlinear coordinate transforms (lat/lon/alt ↔ ECEF/NED)

  ENCODER + IMU FUSION (Mobile Robot Odometry)
  - Wheel encoder: accurate position along path, but accumulates error on slips
  - IMU gyroscope: accurate heading changes
  - Fusing both gives accurate 2D pose [x, y, theta] estimate

5.3 ANALOG SENSOR DENOISING
------------------------------
  - Single-state Kalman filter (scalar): very lightweight, runs on any MCU
  - State: x = true_temperature; Measurement: z = noisy ADC reading
  - A = 1 (temperature changes slowly), H = 1, Q = small, R = ADC noise variance
  - Result: smooth, noise-reduced output with lag tunable via Q/R ratio
  - More adaptive than a simple moving average — responds to real step changes



# SECTION 6: VALIDATION & TESTING


6.1 SIMULATION FIRST (STRONGLY RECOMMENDED)
--------------------------------------------
Before deploying to hardware, validate the filter in software simulation.
This allows controlled experiments and rapid iteration.

  PYTHON WORKFLOW (using FilterPy library)
    from filterpy.kalman import KalmanFilter
    import numpy as np

    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.F = np.array([[1, dt], [0, 1]])   # state transition
    kf.H = np.array([[1, 0]])            # observation
    kf.R = np.array([[sensor_noise]])    # measurement noise
    kf.Q = ...                           # process noise
    kf.x = np.array([[0], [0]])          # initial state
    kf.P = np.eye(2) * 100              # initial covariance (uncertain)

    for z in measurements:
        kf.predict()
        kf.update(z)
        print(kf.x)

  SYNTHETIC NOISE INJECTION
  - Generate ground truth trajectory analytically
  - Add Gaussian noise with known sigma to simulate sensor readings
  - Run filter and compare output to ground truth
  - Vary Q and R to observe filter behavior

  TOOLS
  - Python: FilterPy, NumPy/SciPy
  - MATLAB/Simulink: built-in Kalman filter block, easy visualization
  - C unit test: link filter code against a PC test harness with simulated inputs

6.2 HARDWARE TESTING
---------------------
  LOG AND COMPARE
  - Log raw sensor data and Kalman-filtered output via UART/SWD
  - Plot both — filtered output should be smoother with acceptable lag
  - Compare against a ground truth if available (camera, VICON, reference sensor)

  COMMON FAILURE MODES TO CHECK
  - Filter lag: output too slow to follow real changes → increase Q
  - Filter noise: output still noisy → decrease Q or increase R
  - Filter divergence: output grows unbounded → check P stays positive definite,
    check Q/R values, check matrix implementation for bugs
  - Bias accumulation: steady-state offset → add bias state to model

  DEBUGGING TOOLS
  - Print or log innovation (z - H*x_hat): should be small and zero-mean
  - Print diagonal of P: should decrease toward a steady-state value
  - ARM DWT cycle counter: profile execution time per filter cycle

6.3 PERFORMANCE METRICS
------------------------
  ROOT MEAN SQUARE ERROR (RMSE)
  - RMSE = sqrt(mean((x_true - x_estimated)^2))
  - Lower is better; compare against raw measurement RMSE to quantify improvement
  - Compute per state variable (position RMSE, velocity RMSE, angle RMSE)

  INNOVATION WHITENESS TEST (KEY OPTIMALITY CHECK)
  - The innovation sequence y_k = z_k - H * x_hat_{k|k-1} should be:
    * Zero-mean: E[y_k] = 0
    * White (uncorrelated): E[y_k * y_j^T] = 0 for k ≠ j
    * Consistent: actual innovation covariance ≈ theoretical S_k
  - If innovations are correlated: filter is suboptimal → tune Q or R
  - Test using autocorrelation function (ACF) of innovation sequence
  - Normalized Innovation Squared (NIS): NIS_k = y_k^T * S_k^{-1} * y_k
    should follow a chi-squared distribution with p degrees of freedom

  NORMALIZED ESTIMATION ERROR SQUARED (NEES)
  - NEES_k = (x_true_k - x_hat_k)^T * P_k^{-1} * (x_true_k - x_hat_k)
  - Should follow chi-squared with n degrees of freedom
  - Used to verify that P_k is a correct representation of actual error

  COMPUTATIONAL PROFILING
  - Measure cycles per predict-update cycle using ARM DWT or timer
  - Must complete within sensor period (e.g., 1000 cycles @ 168 MHz = 6 us)
  - Profile matrix inverse (most expensive op); optimize or use steady-state K



# SECTION 7: REFERENCES & RESOURCES


FOUNDATIONAL PAPERS
- Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction
  Problems." Transactions of the ASME, Journal of Basic Engineering.
  (The original 1960 paper — the basis of all Kalman filtering)

- Wan, E.A. & van der Merwe, R. (2000). "The Unscented Kalman Filter for
  Nonlinear Estimation." Proceedings of IEEE Symposium AS-SPCC.
  (Foundation of UKF / sigma-point filters)

BOOKS
- Labbe, R. "Kalman and Bayesian Filters in Python" (free online, GitHub)
  Practical, code-first approach; includes FilterPy library
  https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

- Simon, D. (2006). "Optimal State Estimation: Kalman, H-Infinity, and
  Nonlinear Approaches." Wiley-Interscience.
  Comprehensive theoretical treatment including EKF, UKF, H-inf

- Brown, R.G. & Hwang, P.Y.C. "Introduction to Random Signals and
  Applied Kalman Filtering" (4th Ed). Wiley.
  Classic textbook, good balance of theory and application

ONLINE TUTORIALS
- https://kalmanfilter.net  — equation-by-equation walkthrough with examples
- https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/ — visual intro

CODE LIBRARIES
- FilterPy (Python):   https://github.com/rlabbe/filterpy
  Full KF, EKF, UKF with tests and examples

- TinyEKF (C/Python):  https://github.com/simondlevy/TinyEKF
  Minimal EKF in portable C — designed for embedded systems

- libfixkalman (C):    https://github.com/sunsided/libfixkalman
  Fixed-point Kalman filter for Cortex-M0/M3 without FPU

- SimpleKalmanFilter:  https://github.com/denyssene/SimpleKalmanFilter
  Single-state Arduino-friendly scalar Kalman filter

- CMSIS-DSP (ARM):     https://arm-software.github.io/CMSIS-DSP/
  Optimized matrix ops for ARM Cortex-M (arm_mat_mult_f32, etc.)

ACADEMIC REFERENCE
- Implementation of a C Library of Kalman Filters for Application on
  Embedded Systems. Computers 2022, 11(11), 165. MDPI.
  https://doi.org/10.3390/computers11110165
  (EKF + UKF square-root implementations in C, validated on automotive ECUs)




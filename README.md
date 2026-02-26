# 2DOF-TVC-Rocket-Controller


> Amateur rocket prototype with a **state-feedback LQR attitude control system** for Thrust Vector Control (TVC). The plant is derived **symbolically** from Newton-Euler equations of motion, linearised at the trim point, and augmented with integral action to achieve zero steady-state error. 

<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Finished_model_01.jpg" width="500">

>Finished rocket model.

---

## Repository Structure

```
2DOF-TVC-Rocket-Controller/
│
├── firmware/
│   └── LQR.ino                   # ESP32 flight controller — LQR with integral action
│
├── matlab/
│   ├── Gain_calculate.m          # Symbolic linearisation + DLQR gain computation
│   └── lineal_integrator_lqr.slx # Simulink model — closed-loop step response validation
│
├── docs/
│   └── images/                   # Rocket photos, PCB renders, simulation plots
│
├── LICENSE
└── README.md
```

---

## Overview

This repository contains the embedded flight software and MATLAB/Simulink design toolchain for a **Thrust Vector Control (TVC) rocket**. The system stabilises rocket attitude in pitch and yaw by gimbaling a solid-propellant motor ±12° using two servos.

The control architecture is an **integral-augmented discrete LQR** — a state-feedback controller extended with integral action to track a zero-angle reference with no steady-state error, even in the presence of disturbances.

**Design workflow:**

```
Newton-Euler EOM  →  Symbolic Linearisation  →  Numerical A, B, C, D
                                                         │
                                                  DLQR (DARE solver)
                                                         │
                                              Kx, Ki gains validated in Simulink
                                                         │
                                              Hardcoded into ESP32 firmware
```

---

## System Architecture

```
IMU (Gyro + Accel)
        │
        ▼
  Low-Pass Filters (fc_gyro = 10 Hz, fc_accel = 2 Hz)
        │
        ▼
  Complementary Filter
  (K_gyro = 0.98, K_accel = 0.02)
        │
        ▼
  Coordinate Rotation Matrix
  (IMU mounting offset — 73.8° about roll axis)
        │
        ▼
  State Vector  x = [θ_pitch, θ_yaw, ω_pitch, ω_yaw]ᵀ  (radians)
        │
        ▼
  Integral-Augmented LQR
  u = Ki·∫e dt  −  Kx·x
        │
        ▼
  2-Axis Gimbal  (2× Servo, ±12°, 62°/s rate limit)
```

---

## Control System Design

### 1 — Equations of Motion (`Gain_calculate.m`, Part 1)

The 6-DOF rocket dynamics are derived from first principles using **Newton-Euler equations** with symbolic MATLAB variables, then linearised at the trim point (vertical flight, zero deflection).

**Full 6-DOF state vector:**
```
x = [θ_pitch, θ_yaw, θ_roll, ω_pitch, ω_yaw, ω_roll]ᵀ
```

**Torques acting on the vehicle:**

| Source | Expression |
|---|---|
| TVC thrust torque | τ_T = r_tvc × (F_t · u_γ) |
| Aerodynamic force torque | τ_AF = r_cp × (q·A·[−Cd, Cl_p, Cl_y]ᵀ) |
| Aerodynamic damping moment | τ_AM = q·A·D·[0, Cm_p, Cm_y]ᵀ |

where **u_γ** is the thrust direction unit vector as a function of TVC gimbal angles (γ_pitch, γ_yaw):

```
u_γ = [cos(γ_p)·cos(γ_y),  −sin(γ_p),  cos(γ_p)·sin(γ_y)]ᵀ
```

The full Newton-Euler equation of motion:
```
ω̇ = I⁻¹ · (τ_T + τ_AF + τ_AM  −  ω × (I·ω))
```

The Jacobians **A = ∂f/∂x** and **B = ∂f/∂u** are computed symbolically and evaluated at the equilibrium point (all states and inputs = 0, small-angle approximation). Roll dynamics are decoupled and discarded, yielding a **4×4 pitch-yaw subsystem:**

```
Reduced state:  x = [θ_pitch, θ_yaw, ω_pitch, ω_yaw]ᵀ
Input:          u = [γ_pitch, γ_yaw]ᵀ
```

**Numerical physical parameters:**

| Parameter | Symbol | Value |
|---|---|---|
| Pitch/Yaw moment of inertia | Iyy = Izz | 0.006941 kg·m² |
| Roll moment of inertia | Ixx | 0.00008 kg·m² |
| Nominal thrust | F_t | 12 N |
| CoG to TVC distance | r_tvc | 0.31 m |
| Fuselage diameter | D | 0.085 m |
| Aerodynamic damping (pitch/yaw) | Cm_p, Cm_y | −0.4 |

Iyy was validated experimentally using a **bifilar pendulum** (T = 1.738 s, mean of 10 trials):

```
I = m·g·d²·T² / (16π²·L)
  = 0.723 · 9.81 · 0.054² · 1.738² / (16π² · 0.057)
  = 0.006941 kg·m²
```

---

### 2 — LQR Design (`Gain_calculate.m`, Part 2)

![me](https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Control_system.png)
>Integral-augmented LQR Control system

#### Integral Augmentation

To achieve zero steady-state error without a feedforward term, the state vector is augmented with the **accumulated tracking error:**

```
ξ̇ = e = r − C·x    (ξ is the integral of tracking error)
```

The augmented discrete system (ZOH, sample time Ts):

```
x_aug = [x; ξ]     (6 states: 4 plant + 2 integrators)

A_aug = [ A_d   |    0  ]       B_aug = [ B_d ]
        [−Ts·C  |  I_2×2]               [  0  ]
```

#### DLQR Gain Computation

The discrete LQR problem is solved via the **Discrete Algebraic Riccati Equation (DARE):**

```matlab
K_aug = dlqr(A_aug, B_aug, Q_aug, R)
```

with weighting matrices:
```
Qx = diag([10, 10, 0.0001, 0.0001])   % Penalise angle error >> rate error
Qi = diag([300, 300])                  % Strong integral action
R  = 1.5 · I₂                         % Moderate actuator effort penalty
```

The augmented gain matrix is split into:
```
K_aug = [Kx | −Ki]

Kx (2×4) — state feedback gain
Ki (2×2) — integral gain
```

**Control law (implemented in firmware):**
```
u = Ki·ξ − Kx·x
```

**Current gains (hardcoded in `LQR.ino`):**
```
Kx = [[ 0.0,    −84.1365,  0.0,    −15.4607],
      [−84.1365,  0.0,    −15.4607,  0.0    ]]

Ki = [[ 0.1813,  0.8081],
      [ 0.3882, −0.1225]]
```

Closed-loop stability is verified automatically at the end of the script — all discrete eigenvalues must lie strictly inside the unit circle.

---

### 3 — Simulink Validation (`lineal_integrator_lqr.slx`)
![me](https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Plant.png)

>Simulink model.

Before flashing to hardware, the closed-loop response is validated in Simulink against a 10° and 20º step input. The model reproduces the full augmented discrete system including  position saturation (±12°).

<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/system_response.jpg" width="500">

>System response to a 20º and 10º steps.

<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/actuator_response.jpg" width="500">

>Actuator response to a 20º and 10º steps.

Once the step response meets the performance requirements, the `Kx` and `Ki` values are pasted into `LQR.ino`.

---

### 4 — How to Compute New Gains

1. Open `matlab/Gain_calculate.m` in MATLAB *(Symbolic Math Toolbox required)*
2. Adjust the `val_*` variables to match your rocket's physical parameters
3. Tune `Qx`, `Qi`, and `R` to trade off response aggressiveness vs. actuator effort
4. Run the script — it prints `Kx` and `Ki` and confirms closed-loop stability
5. Validate the step response in `matlab/lineal_integrator_lqr.slx`
6. Paste the verified gains into `firmware/LQR.ino`:

```cpp
float Kx[2][4] = { {0.0, -84.1365, 0.0, -15.4607},
                   {-84.1365, 0.0, -15.4607, 0.0} };
float Ki[2][2] = { {0.1813, 0.8081},
                   {0.3882, -0.1225} };
```

---

## Firmware (`firmware/LQR.ino`)

### Sensor Fusion

Pitch and yaw are estimated by a **complementary filter** fusing gyroscope integration with accelerometer tilt:

```cpp
pitch = (1 - alpha) * pitchFromGyro + alpha * pitch_acc_deg;  // alpha = 0.02
```

The gyroscope dominates (98%) to suppress accelerometer noise during the thrust phase. Low-pass filters are applied independently to each sensor axis before fusion (fc_gyro = 10 Hz, fc_accel = 2 Hz).

The Madgwick filter was evaluated and discarded due to: (1) unreliable magnetometer readings in the airframe environment, and (2) DFRobot library incompatibility with ESP32 compilation targets. The complementary filter offers equivalent accuracy at a fraction of the computational cost.

### Coordinate Transformation

The IMU is physically mounted rotated 73.8° from the rocket's roll axis. A real-time rotation matrix realigns sensor axes with rocket body axes:

```cpp
pitch_rocket =  cos(theta_mount_rad) * pitch_s_deg + sin(theta_mount_rad) * yaw_s_deg;
yaw_rocket   = -sin(theta_mount_rad) * pitch_s_deg + cos(theta_mount_rad) * yaw_s_deg;
```

### LQR Control Loop

```cpp
void LQRCompute(float dt, float x[4], float output[2]) {
  error[i]    = setpoint[i] - x[i];
  integral[i] += error[i] * dt;

  output[i] = Ki * integral  -  Kx * x;

  // Anti-windup: revert integral accumulation if output saturates at ±12°
}
```

The state vector fed to `LQRCompute` is always in radians:
```cpp
float x[4] = {
  DEG2RAD(pitch_rocket),   // θ_pitch
  DEG2RAD(yaw_rocket),     // θ_yaw
  DEG2RAD(gy_filt),        // ω_pitch
  DEG2RAD(gx_filt)         // ω_yaw
};
```

---

## Hardware

| Component | Part |
|---|---|
| Microcontroller | ESP32 WROOM-32D |
| IMU | DFRobot Fermion 10DOF (ITG-3200 gyro + ADXL345 accel) |
| Actuators | 2× servo motors (pitch & yaw axes) |
| PCB | Custom KiCad shield — FR4 TG135, 1oz Cu, HASL finish |
| Power | LiPo 2S 1000mAh + 4A Buck Converter (XT30 connector) |
| Data logging | MicroSD (SPI) |
| Motor | 24mm solid propellant — Estes E12 / Aerotech F12 / Cesaroni F51 |
| Structure | 3D-printed PLA + reinforced cardboard fuselage, Ø85mm |

The PCB was designed in KiCad and manufactured externally. Track widths on the power path were calculated per IPC-2221 for 4A peak current. I²C is used for the IMU, SPI for the microSD.

<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/PCB_Render.png" width="300">

>Rendered PCB designed in KiCAD.

<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/PCB_Layout.png" width="300">

>PCB Layout.

![me](https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/esp32.png)
>ESP32 WROOM32D Pinout.

![me](https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/fermion.png)
>DFROBOT FERMION 10 dof IMU sensor.


---
## Gimbal Mechanism
![me](https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/actuation_mechanism.gif)
![me](https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/actuation_system.gif)
>2 DOF actuation mechanism.

---
## Installation

### Dependencies (Arduino IDE)

```
ESP32Servo
Wire              (built-in)
ITG3200           (DFRobot)
DFRobot_ADXL345
```

### Calibration

Update the constants at the top of `LQR.ino`:

```cpp
float gyro_bias_dps[3] = { -4.2154f,  0.8157f, -0.6550f };  // auto-recomputed at startup
float accel_offset[3]  = {  4.969f,  -6.378f,  -37.069f };
float accel_g[3]       = { 1.0/262.561f, 1.0/262.578f, 1.0/259.099f };
float theta_mount_rad  = (90.0 - 16.2) * (PI / 180.0);       // IMU mounting angle
```

Gyro bias is **automatically recomputed at every startup** from 500 samples (~1.5 s). Servo centre positions are adjustable via `SERVO_CENTER_P` and `SERVO_CENTER_Y` to compensate manufacturing tolerances.

### Activation

LQR is activated by holding the button on GPIO 14 (active LOW). Status LEDs on GPIO 12 (OK) and 13 (ERR) indicate system state. Serial debug output at 115200 baud streams all sensor and control values at 10 Hz when `debug = true`.

---

## Project Metrics

| Parameter | Value |
|---|---|
| Total budget | €270 |
| Timeline | 16 months (Sept 2024 – Jan 2026) |
| Design iterations | 3 full prototypes |
| Rocket mass | 0.723 kg |
| Fuselage diameter | Ø85 mm |
| Moment of inertia (pitch/yaw) | 0.00694 kg·m² (bifilar pendulum) |
| Control sample time | ~0.02 s (50 Hz) |
| Max gimbal deflection | ±12° |
| Servo rate saturation | 62°/s |

---

## Finished model


<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Finished_model_02.jpg" width="500">
<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Finished_model_03.jpg" width="500">
<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Finished_model_04.jpg" width="500">
<img src="https://github.com/antonpatino/2DOF-TVC-Rocket-Controller/blob/main/images/Finished_model_05.jpg" width="500">

>Finished model images.

---



## Author

**Antón Patiño Carro** — B.Sc. Mechanical Engineering, University of A Coruña (UDC)  
anton.patino.career@gmail.com  
[linkedin.com/in/anton-patino-carro](https://linkedin.com/in/anton-patino-carro) · [github.com/antonpatino](https://github.com/antonpatino)

---

## License

MIT — see [LICENSE](LICENSE) for details.

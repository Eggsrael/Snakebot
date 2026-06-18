# SnakeBot: Leader-Follower Autonomous Robot System

**CSE 598 · Arizona State University**

A two-robot leader-follower system running on Raspberry Pi 5. The follower robot tracks a leader at a target distance using three VL53L0X time-of-flight sensors and a PPO policy trained in a custom Gymnasium simulation, with L298N motor drivers for differential-drive actuation.

---

## System Overview

```
Leader robot  ──(moves freely)──▶  Follower robot
                                       │
                               3× VL53L0X ToF (±25°, 0°)
                                       │
                               PPO policy (obs: ToF + relative pose)
                                       │
                               L298N + RPi.GPIO (BCM)
                                       │
                               Left / Right DC motors (PWM)
```

The follower has no knowledge of the leader's future path. It observes only its own ToF sensor readings and the relative distance and bearing to the leader, then selects one of five discrete actions: forward, forward-left, forward-right, backward, or stop.

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Compute | Raspberry Pi 5 | Runs inference + motor control |
| Motor driver | L298N H-bridge | BCM pins 5, 6, 13, 19 (direction); pin 24 (PWM enable) |
| Distance sensors | 3× VL53L0X ToF | Left (−25°), center (0°), right (+25°) from heading |
| IMU | MPU6050 | Gyroscope data for turn-state classification as RL state features |
| Drive | 2× DC motors | Differential drive; PWM duty cycle control |
| Communication | I2C | VL53L0X sensors via Adafruit VL53L0X library; MPU6050 on same bus |

---

## Software Architecture

### `motor_control.py` - Hardware abstraction

`Motor` class wraps RPi.GPIO in BCM mode. Exposes `move_forward(rduty, lduty)`, `move_backward()`, `move_left()`, `move_right()`, `stop()`, and `cleanup()`. PWM runs at 100 Hz; duty cycle controls speed independently per wheel.

```python
motor = Motor()
motor.move_forward(rduty=80, lduty=80)
```

### `train_follower.py` - Simulation + PPO training

Custom `gymnasium` environment (`FollowerEnv`) modeling the follower in a 5×5m 2D world with three rectangular obstacles. The leader follows a scripted sinusoidal path (`0.07 m/s`, `ω = 0.5·sin(phase)`).

**Observation space (7-dim):**

| Index | Feature | Range |
|---|---|---|
| 0–2 | ToF left / center / right (simulated ray cast ±25°) | [0, 1.5] m |
| 3 | Relative distance to leader | [0, 10] m |
| 4 | Relative bearing to leader | [−π, π] rad |
| 5 | Yaw rate hint | [−2, 2] |
| 6 | Previous action (normalized) | [0, 1] |

**Action space (Discrete 5):**

| Action | Behavior |
|---|---|
| 0 | Forward (v=0.08 m/s) |
| 1 | Forward-left (v=0.06, ω=+0.5) |
| 2 | Forward-right (v=0.06, ω=−0.5) |
| 3 | Backward (v=−0.06) |
| 4 | Stop |

**Reward shaping:**
- Primary: `−2.5·|dist − 0.5| − 1.2·|bearing|` (target follow distance = 0.5 m)
- Progress bonuses for closing distance and bearing error each step
- +0.2 alive bonus per step
- −0.05 action-change penalty (smoothness)
- −1.0 if any ToF reading < 0.2 m (obstacle proximity)
- −8.0 + termination if distance > 2.5 m or |bearing| > 100°
- −20.0 + termination on collision
- +0.8 success bonus if within 0.08 m of target distance and 8° of heading

**Training:**

```python
model = PPO(
    "MlpPolicy", env,
    learning_rate=3e-4, n_steps=1024, batch_size=64,
    gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.01,
)
model.learn(total_timesteps=150_000)
model.save("ppo_follower")
```

Trained model saved as `ppo_follower.zip` (included in repo).

### `testAll.py` / `test_train_org.py` - Evaluation

Runs the saved PPO policy (`ppo_follower.zip`) in the simulation environment for 300 steps, logging action, per-step reward, and cumulative reward.

### `driveTest/` - Hardware integration tests

Motor drive test scripts for verifying L298N wiring and PWM response on physical hardware before deploying the trained policy.

### `tof_test/` - Sensor validation

VL53L0X I2C arbitration and reading validation. Three sensors share the I2C bus via Adafruit VL53L0X library; this directory contains the per-sensor verification scripts.

---

## Setup

### Simulation (no hardware required)

```bash
git clone https://github.com/Eggsrael/Snakebot
cd Snakebot
pip install gymnasium stable-baselines3 numpy
```

**Train:**
```bash
python train_follower.py
# Saves ppo_follower.zip
```

**Test saved model:**
```bash
python testAll.py
```

### On-hardware (Raspberry Pi 5)

```bash
pip install RPi.GPIO adafruit-circuitpython-vl53l0x smbus2
```

Wire L298N per BCM pin assignments in `motor_control.py`:

| BCM Pin | Signal |
|---|---|
| 5 | Right motor IN1 |
| 6 | Right motor IN2 |
| 24 | Right motor EN (PWM) |
| 13 | Left motor IN1 |
| 19 | Left motor IN2 |
| 24 | Left motor EN (PWM) |

Wire VL53L0X sensors to I2C (SDA/SCL) with XSHUT pins for address arbitration if running all three simultaneously.

**Run hardware motor test:**
```bash
python main.py
# Calls driveTest/test.Motor()
```

---

## Repository Structure

```
Snakebot/
├── train_follower.py     # Gymnasium env + PPO training
├── motor_control.py      # L298N / RPi.GPIO motor abstraction
├── testAll.py            # Policy evaluation in simulation
├── test_train_org.py     # Alternate evaluation / training script
├── main.py               # Hardware entry point (driveTest)
├── ppo_follower.zip      # Trained PPO policy (Stable-Baselines3)
├── driveTest/            # Motor hardware verification scripts
└── tof_test/             # VL53L0X I2C validation scripts
```

---

## Notes

- The simulation target follow distance (0.5 m) differs from the hardware target reported in project documentation (~10 cm). The sim uses a larger value to make the training task tractable in the simplified 2D world; the physical gap regulation relies on direct ToF thresholding in the hardware control loop.
- The L298N enable pins for both motors share BCM pin 24 in the current `motor_control.py` if independent speed control per motor is needed, wire separate PWM pins and update the constructor arguments.
- I2C bus contention between three VL53L0X sensors at their default address (0x29) requires XSHUT-based re-addressing at startup; see `tof_test/` for the arbitration sequence.

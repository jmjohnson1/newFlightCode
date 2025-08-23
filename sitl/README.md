# Quadcopter SITL (Software-In-The-Loop) Simulation

This SITL simulation allows you to run your Teensy quadcopter flight code on your PC for debugging and testing without hardware.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Flight Code   │    │  Hardware HAL   │    │ Sensor Models   │
│   (Your Code)   │◄──►│   (teensy_hal)  │◄──►│   (IMU, RC)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │                        │
                                ▼                        ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │ Physics Engine  │◄──►│   Environment   │
                       │ (Your Physics)  │    │   (Wind, etc.)  │
                       └─────────────────┘    └─────────────────┘
```

## Building

### Prerequisites
```bash
# Ubuntu/Debian
sudo apt-get install build-essential cmake libeigen3-dev

# macOS
brew install cmake eigen

# Optional visualization
sudo apt-get install libglfw3-dev libgl1-mesa-dev  # Ubuntu
brew install glfw  # macOS
```

### Build Steps
```bash
cd sitl
mkdir build && cd build
cmake ..
make -j4
```

## Usage

### Basic Simulation
```bash
./sitl_quadcopter
```

### With Custom Configuration
```bash
./sitl_quadcopter --config ../config/sitl_config.yaml
```

### Testing Scenarios
```bash
# Hover test
./sitl_quadcopter --scenario hover_test

# Step response test  
./sitl_quadcopter --scenario step_response

# Sine wave tracking
./sitl_quadcopter --scenario sine_tracking
```

## Integration with Your Flight Code

### 1. Modify Your Flight Code Headers
Add this to your main flight code files:

```cpp
#ifdef SITL_BUILD
#include "teensy_hal.h"
#else
#include <Arduino.h>
// ... your normal Teensy includes
#endif
```

### 2. Replace IMU Sensor Calls
In your `IMU.cpp`, replace hardware sensor reads:

```cpp
#ifdef SITL_BUILD
// Use simulated sensor data
extern float mock_accel_x, mock_accel_y, mock_accel_z;
extern float mock_gyro_x, mock_gyro_y, mock_gyro_z;

void mpu6050::Update() {
    // Get simulated data instead of I2C reads
    accXRaw_ = mock_accel_x;
    accYRaw_ = mock_accel_y;
    accZRaw_ = mock_accel_z;
    
    gyroXRaw_ = mock_gyro_x;
    gyroYRaw_ = mock_gyro_y;
    gyroZRaw_ = mock_gyro_z;
    
    // Apply your existing filtering
    accX_ = butterworth2_apply(&accelFilter_1, accXRaw_);
    // ... etc
}
#else
// Your existing hardware code
#endif
```

### 3. Integrate Your Physics Engine
Replace the TODO sections in `sitl_main.cpp`:

```cpp
#include "your_physics_engine.h"

// In initializeSITL():
physics_sim = new YourPhysicsEngine();

// In simulationStep():
physics_sim->update(dt, motor_commands);
auto state = physics_sim->getState();

// Update sensor models with physics data
Eigen::Vector3d true_accel_body = physics_sim->getAcceleration();
Eigen::Vector3d true_gyro_body = physics_sim->getAngularVelocity();
imu_sim->update(dt, true_accel_body, true_gyro_body);
```

### 4. Connect Motor Commands
Capture motor commands from your flight code:

```cpp
// In your motor command function:
#ifdef SITL_BUILD
extern void setSITLMotorCommands(float m1, float m2, float m3, float m4);
#endif

void Motors::CommandMotor() {
#ifdef SITL_BUILD
    setSITLMotorCommands(motorCommands_[0], motorCommands_[1], 
                         motorCommands_[2], motorCommands_[3]);
#else
    // Your existing PWM output code
#endif
}
```

## Debugging Features

### Real-time Logging
All data is logged to `sitl_log.csv` with timestamps:
- Position, velocity, attitude
- Sensor readings (raw and filtered)  
- Motor commands
- PID outputs
- Control inputs

### Console Output
Enable verbose logging in the config to see:
- Loop timing information
- PID values in real-time
- Sensor readings
- Flight mode changes

### Integration with Analysis Tools
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load SITL log data
data = pd.read_csv('sitl_log.csv')

# Plot attitude response
plt.figure(figsize=(12, 4))
plt.subplot(131)
plt.plot(data['time'], data['roll'])
plt.title('Roll Response')

plt.subplot(132) 
plt.plot(data['time'], data['pitch'])
plt.title('Pitch Response')

plt.subplot(133)
plt.plot(data['time'], data['yaw'])  
plt.title('Yaw Response')

plt.tight_layout()
plt.show()
```

## Testing Your Jitter Issue

### 1. Compare Timing
Enable timing logs to compare loop execution times:
```cpp
#ifdef SITL_BUILD
static uint32_t last_loop_time = micros();
uint32_t current_loop_time = micros();
if (current_loop_time - last_loop_time > 600) { // > 500μs + margin
    printf("Loop timing violation: %d μs\n", current_loop_time - last_loop_time);
}
last_loop_time = current_loop_time;
#endif
```

### 2. Step Response Tests
Use the step response scenario to test PID tuning:
```bash
./sitl_quadcopter --scenario step_response
```

### 3. Frequency Analysis
Log high-rate data and analyze for oscillations:
```python
# Look for high-frequency oscillations in motor commands
fft = np.fft.fft(data['motor1'])
freqs = np.fft.fftfreq(len(data), 1/1000)  # 1000 Hz sample rate
plt.semilogy(freqs[:len(freqs)//2], np.abs(fft[:len(fft)//2]))
```

This SITL setup will let you run your exact flight code with simulated sensors and your physics engine, making it much easier to debug timing issues and test PID parameters safely!
# esp32-rover-firmware

ESP32 rover firmware in C/C++: sensor interfacing, wireless control, and
real-time motor control. Built during a NASA California Space Grant Summer Internship.

## My Role

Delivered firmware implementation (C/C++) in a 4-person team and led further improvements covering rover movement, sensor integration, and wireless telemetry

## System Overview

Controller ESP32 reads two joysticks and buttons, then sends a control packet to the rover over ESP-NOW every 500 ms. Second ESP32 on the rover drives mecanum wheel motors, reads sensors, sends a telemetry packet to Controller ESP32, and controls a 3 servo arm for grip, lift, and tilt.

**Sensors:** BME280 and Capacitive Soil Moisture Sensor

**Power:** Separate supplies for motors/servos, and controller

## Communication Design

<img width="1920" height="1080" alt="System diagram" src="https://github.com/user-attachments/assets/3c61047c-8241-4900-b643-acb7b76f11c6" />

- **Control packet from controller -> rover**: joystick positions, button states, and sensor toggle flags
- **Telemetry packet from rover -> controller**: latest sensor readings, only sent to the controller when sensors are enabled

## Problems & Fixes

1. **Arm and rover could not move at the same time:** added nonblocking timing where every task runs on it's own interval.

2. **Sensor LEDs flickering while button was held:** holding button for half a second flickered and sensor ends up in random state when let go. Added edge detection so it only acts at the moment the button goes from released to pressed. Also debouncing between pressed and released for a few milliseconds before settling which waits 20 ms and checks again to make sure it was a real press.

3. **Power issues under load:** when programmed to have multiple servos and motors moving at once, there were ESP32 shutdowns and motors/servos stalled as the current supply couldn't keep up with the current spikes. Moved the actuators to a higher-capacity external supply (5V, up to 3A) with controller on separate power. 

## Live Sensor Dashboard
<img width="1919" height="1140" alt="Sensor Dashboard" src="https://github.com/user-attachments/assets/226c0269-ac9e-4435-bf32-10822c2fafc9" />

Added a webserver to display live sensor data remotely and toggle states at 2 Hz refresh. Hover any point on each graph to check the exact reading from beginning to current time.

## Controller Iterations
<img width="1280" height="720" alt="Controller Stages" src="https://github.com/user-attachments/assets/4ed21667-7108-4c1b-a5ac-42b14b1837e7" />

1. basic arm control and joystick
2. added a mode button, green for rover and red for arm, one-at-a-time movement and revealed jitter
3. non-blocking loop, simultaneous rover and arm motion, remote sensor toggling, and telemetry 

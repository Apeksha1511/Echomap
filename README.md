# EchoMap

## Real-Time Indoor Navigation for Visually Impaired Using Ultrasonic SLAM and Personalized Confidence-Based Path Planning on Raspberry Pi

EchoMap is a low-cost, offline indoor navigation system designed to assist visually impaired users using a Raspberry Pi and three ultrasonic sensors.

The system detects obstacles in the user's surroundings, creates an occupancy-grid representation of an indoor environment, stores landmarks, and provides destination-based navigation using A* path planning and offline text-to-speech.

The system does not require internet connectivity, cameras, or a preloaded floor plan.

---

## Project Overview

EchoMap operates through three modes:

### 1. Discovery Mode

Discovery Mode is the default mode when the Raspberry Pi starts.

Three HC-SR04 ultrasonic sensors continuously monitor:

- Left
- Center
- Right

The system provides audio guidance such as:

- Stop
- Move Left
- Move Right
- Turn Left
- Turn Right
- Slow Down

Speech recognition is not continuously active in Discovery Mode. This prevents unwanted voice commands from being detected in crowded environments.

---

### 2. Mapping Mode

Mapping Mode is intended for a sighted person who assists in creating the indoor map.

A long press of Button 1 switches from Discovery Mode to Mapping Mode.

During Mapping Mode:

- Ultrasonic sensors observe the environment.
- The occupancy grid is updated.
- The sighted mapper provides intentional movement commands.
- Voice commands can be used for movement and turns.
- Landmarks can be registered using Button 1.
- Landmark names are captured using offline speech recognition.
- Landmark information is stored in SQLite.

Supported movement/turn commands include:

- Turn Left
- Turn Right
- Move Left
- Move Right
- Move Forward / Walk Forward

The system uses explicit movement commands because the current implementation does not depend on an IMU for continuous movement tracking.

---

### 3. Navigation Mode

Navigation Mode allows the user to select a previously stored landmark as the destination.

Navigation workflow:

1. Press Button 2 to enter destination selection.
2. Press Button 2 to cycle through stored landmarks.
3. Press Button 3 to select the destination.
4. A* path planning calculates a route.
5. The route is converted into step-based audio instructions.
6. Ultrasonic sensors continue to provide real-time obstacle safety.
7. Unexpected obstacles can trigger warnings and route replanning.

Example guidance:

"Walk 6 steps."

"Turn right."

"Walk 8 steps."

---

## Hardware

- Raspberry Pi 3
- 3 × HC-SR04 Ultrasonic Sensors
- Voltage-divider circuits for HC-SR04 Echo pins
- Push Buttons
- Microphone for Mapping Mode speech input
- Speaker / audio output
- Optional wearable mounting arrangement

The MPU6050 is not required for the current system.

---

## Ultrasonic Sensor GPIO Connections

| Sensor | TRIG | ECHO |
|---|---:|---:|
| Left | GPIO 22 | GPIO 25 |
| Center | GPIO 17 | GPIO 27 |
| Right | GPIO 23 | GPIO 24 |

The center sensor is positioned around waist level, while the left and right sensors are positioned higher to improve coverage of lateral and upper obstacles.

---

## Software Architecture

```text
3 × HC-SR04 Sensors
        |
        v
Distance Processing
        |
        v
Obstacle Detection
        |
        v
Dynamic Obstacle Classification
        |
        v
2D Occupancy Grid
        |
        +---------> SQLite Landmark Database
        |
        v
A* Path Planner
        |
        v
Navigation Instructions
        |
        v
Offline Text-to-Speech
        |
        v
User Guidance
        |
        v
Session Metrics
        |
        v
Confidence / Progression
        |
        v
Flask Dashboard

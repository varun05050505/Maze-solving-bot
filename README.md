# Line Following & Maze Solving Robot

![Bot](assets/bot.jpeg)

This project implements an **autonomous line-following and maze-solving robot** built on the **ESP-IDF** framework using **ESP-IDF**.  
The robot is capable of:

- Following a black line using a PID-based control system  
- Detecting and navigating junctions (left, right, T, and U-turns)  
- Detecting objects using an IR sensor and **pushing them** forward  
- Detecting the **end of the maze** and stopping precisely  
- Real-time PID tuning through an integrated **HTTP tuning server**  

---

## 🧠 Project Overview

The robot uses a **5-channel line sensor array** to read surface reflectance values. These readings are normalized, processed, and used to calculate an **error signal** which drives a PID controller.  
The PID output adjusts motor speeds dynamically to keep the bot centered on the line.

In addition to line following, the bot intelligently handles:
- **T-junctions**, **left turns**, **right turns**, and **U-turns**
- **End detection** to stop at maze completion
- **Box detection** using an IR proximity sensor (GPIO input)
- Automatic **push mechanism** when a box is detected on its path

---


## 🧮 Control Logic

### 1. Line Sensor Normalization
Sensor values are mapped between `LSA_MIN` and `LSA_MAX` and inverted for consistent logic.

### 2. PID Control
The robot computes:
- `error` — deviation from the center of the line  
- `correction` — proportional + integral + derivative adjustments  

Motor speeds are tuned as:

left_duty = OPT_DUTY + correction<br>
right_duty = OPT_DUTY - correction

### 3. Object Detection & Push Logic
When IR sensor (GPIO 0) detects an object:
- Robot moves forward briefly to push the box
- Reverses and realigns itself on the track

### 4. End Detection
After executing a left turn, the robot verifies if it has reached the end of the path using line sensor conditions and timing.

---

## 🧰 Build & Flash Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/varun05050505/Maze-solving-bot.git

2. Set up the ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh

3. Flash the code

    ``` bash
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor

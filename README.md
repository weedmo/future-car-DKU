# 3rd Future Automotive Autonomous Driving Competition - Dankook University

This repository contains the autonomous driving code and mission logic developed for the **3rd Future Automotive Autonomous Driving Competition**, hosted in 2024.  
The challenge involved completing multiple mission stages using a limited set of sensors under strict environmental and timing conditions.

🔗 GitHub Repo: https://github.com/weedmo/future-car-dankook-univ

---

## 🎯 Overview

This competition limited the type and number of sensors available for all teams.

- **Allowed Sensors**:
  - 1 × LiDAR
  - 2 × Cameras

### 🏁 Competition Structure

1. **Basic Driving Mission**  
   → Top 10 teams with fastest lap times (2 laps) qualify for the next round.

2. **Main Missions** (combined score ranking):
   - Obstacle Avoidance
   - Traffic Light Response
   - Autonomous Parking

---

## 👨‍💻 Role & Contributions

- 🧠 **Team Lead**
- 🧭 Obstacle Avoidance Logic
- 🅿️ Autonomous Parking Algorithm
- 🧾 Data Collection & YOLOv8 Labeling for Lane Detection

---

## 🧰 Technical Approach

### 1. Lane Detection  
Due to limited sensors, traditional lane detection was not viable.  
Instead, we:
- Recorded a bag file of the track
- Annotated images manually
- Trained a **YOLOv8 segmentation model** to detect lane regions

➡️ As training data increased, lane tracking stability improved significantly.

---

### 2. Obstacle Avoidance  
- Defined a forward-facing ROI on LiDAR data
- Implemented smooth avoidance behavior upon detecting objects within a threshold range

---

### 3. Traffic Light Mission  
- Used **YOLOv8** object detection
- Detected red/green states and triggered corresponding vehicle actions (stop/go)

---

### 4. Parking Mission  
- Used a right-side ROI window to detect free space
- Performed hardcoded parking maneuvers based on vehicle position and spacing

---

## 🎬 Demo Videos

### ▶️ Basic Driving Mission  
[![Watch Basic Driving](https://img.youtube.com/vi/FwGlec1eLXw/0.jpg)](https://youtu.be/FwGlec1eLXw?t=3596)  
> Starts at 59:56

### ▶️ Mission Execution (Obstacle, Traffic Light, Parking)  
[![Watch Mission Driving](https://img.youtube.com/vi/FwGlec1eLXw/0.jpg)](https://youtu.be/FwGlec1eLXw?t=7067)  
> Starts at 1:57:47

---

## 📝 Reflections

Despite long preparation, we were limited by sensor count and hardware power.  
- Our vehicle was relatively light but the motor performance lagged compared to other teams.
- In the first stage, we barely qualified due to speed limitations.
- In the final stage, we **successfully completed all missions** including obstacle avoidance, traffic light handling, and parking.

However, we only received the Bronze Prize.

> After the competition, we found out many teams had **force-overridden motor output limits** — something we consciously avoided due to safety and competition fairness.  
> While it was disappointing, we take pride in building one of the **most reliable and safest autonomous systems** in the competition.


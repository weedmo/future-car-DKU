# 3rd Future Automotive Autonomous Driving Competition - Dankook University

This repository contains the autonomous driving code and mission logic developed for the **3rd Future Automotive Autonomous Driving Competition**, hosted in 2024.  
The challenge involved completing multiple mission stages using a limited set of sensors under strict environmental and timing conditions.

---

## 🏆 Award & Media Coverage

🥉 **Bronze Prize Winner** – 3rd Future Automotive Autonomous Driving Competition (2024)

📸 Featured in Dankook University News:  
[Click to read full article (Korean)](https://www.dankook.ac.kr/web/kor/dku-today?p_p_id=Bbs_WAR_bbsportlet&p_p_lifecycle=0&p_p_state=normal&p_p_mode=view&_Bbs_WAR_bbsportlet_action=view_message&_Bbs_WAR_bbsportlet_messageId=790642)

> "Even though we didn't modify the motor output like other teams did, we safely completed every mission with a stable system. That’s something we’re truly proud of."  
> — From the team interview in the DKU article

![Award Ceremony](https://path-to-your-award-photo-if-you-have-it.jpg) <!-- 이미지가 깃허브에 올라온 경우 -->


---

## 🎯 Overview

This competition limited the type and number of sensors available for all teams.
![KakaoTalk_20250307_190416478](https://github.com/user-attachments/assets/fb57aef1-c4dc-4b06-a1dd-5e2f508bbdcf)

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


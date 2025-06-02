# 41118_SortBot

An intelligent robotic sorting system built in PyBullet that combines deep reinforcement learning (PPO) with object detection (YOLOv8). The uArm robot detects, picks up, and sorts coloured boxes into their designated zones autonomously.

---

## Team Composition

| Name         | Role                     |
|--------------|--------------------------|
| Minh K. Nguyen  | Simulation, Robot control, PPO, YOLO, Vision, integration & deployment |
| Lauren Seeto | Robot control, Robot Modelling, PPO             |
| Michele Liang | Vision, YOLO, documentation            |
|Luis Pratama| Vision, YOLO video director, voice, editor|
|Matthew Truong| Simulation Modelling|

> This project was developed as part of the 41118 Artificial Intelligence in Robotics unit at UTS.
> Special thanks to Dr. Raphael Falque for his guidance and teaching throughout this subject.
---

## Installation Instructions


# Clone the repository
## HTTPS:
```bash
git clone https://github.com/yourusername/41118_SortBot.git
```
## ssh:
```
git clone git@github.com:FattDuckk/ai_project.git
```

# Running
```bash
cd main/UArm_robot_training
```
```bash
python3 sort.py
```


# Dependencies
### Install PPO 
```bash
pip install stable_baselines3
```

### Install YOLOv8 
```bash
pip install ultralytics
```

### Install PyBullet 
```bash
pip install pybullet
```

### Install Numpy
```bash
pip install numpy
```

### Install Gym
```bash
pip install gym
```

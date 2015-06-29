# Emotional-manager

Emotion-action framework for nao. Takes a point in a 2D map (valence and arousal) as input and shows a behaviour according to it. Modifies arms, torso and head positions, as well as eyes LED colors and speech pitch and volume.

## Installation

Install as any other ROS package

## Dependencies

- [dlib](http://dlib.net/) for the 2D head pose estimation in the vision module,
- [OpenCV](http://opencv.org/downloads.html) for the visual features cap

## Usage

Set NAO's ip adress in the action_manager.py and the emotional_manager.py

Execute: `roslaunch emotional_manager nao_emotional.launch`


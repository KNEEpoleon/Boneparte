# BONE.P.A.R.T.E. (Bone Precision Augmented Reality Tracking Equipment)

This repository houses the core infrastructure for **BONE.P.A.R.T.E.**, a surgical robotics system focused on enhancing orthopedic procedures using robotic manipulation, real-time perception, and augmented reality guidance.

## Project Overview

BONE.P.A.R.T.E. is designed to assist surgeons during pin placement precision task in an orthopedic surgery. The system combines:

- **Real-time pose estimation** of surgical targets (e.g., bones, guides)
- **Robotic planning and manipulation** to align tools with target poses
- **Augmented reality interfaces** for intuitive visualization and intra-op feedback

Our goals are higher surgical accuracy, reduced operating time, and better outcomes through automation and AR integration.

## Repository Structure

Boneparte/
├── Build/ (After colcon build)
├── Install/ (After colcon build)
├── Logs/ (After colcon build)
├── Docker/
├── src/
│   ├── perception/       # Subsystem for detecting and tracking surgical targets
│   ├── AVP/              # Apple Vision Pro subsystem
│   └── manipulation/     # Robot control for tool alignment and interaction
├── README.md             
└── LICENSE


Each subdirectory under `src/` contains its own `README.md` for deeper technical details.

## Setup Instructions

Can be found inside the src folder
└──src/README.md
## Contributors
- [Sreeharsha Paruchuri](https://github.com/sreeharshaparuchur1)
- [Parth Singh](https://github.com/parths5)
- [Daksh Adhar](https://github.com/a-daksh)
- []()
- []()

## Sponsors

Developed as part of the CMU MRSD Capstone with support from [Smith + Nephew](www.smith-nephew.com)

## License



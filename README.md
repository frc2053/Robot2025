[![Build Robot Code](https://github.com/frc2053/Robot2025/actions/workflows/build.yml/badge.svg)](https://github.com/frc2053/Robot2025/actions/workflows/build.yml)
[![Formatting](https://github.com/frc2053/Robot2025/actions/workflows/format.yml/badge.svg)](https://github.com/frc2053/Robot2025/actions/workflows/format.yml)
[![Sanitizers](https://github.com/frc2053/Robot2025/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/frc2053/Robot2025/actions/workflows/sanitizers.yml)

# FRC 2053 Southern Tier Robotics 2025 Code

This repo holds our code for the 2025 Robot. Our test drivebase is a swerve drivetrain using Krakens for Drive and V3 Falcon 500's for steer with the MK4i modules geared at the "L2" option. We have a pigeon v2 as our IMU as well all running through the CANivore. Our library uses Phoenix V6 Pro.

# How to setup

## Prerequisites (Install this stuff first)
- Python 3 (not installed from Windows store. Please install from Python website)
- Visual Studio 2022 with C++ workload
- [Latest Stable 2025 WPILib Version](https://github.com/wpilibsuite/allwpilib/releases)

## Install steps: 
- Create your virtual environment
    Inside the root of the project run:
    `python -m venv ./venv`
- Activate your virtual environment
    `./venv/Scripts/Activate.ps1`
- Install the code formatter
    `pip install -r requirements.txt`

## Build steps:
- To build the code, press Ctrl+Shift+P and search for "Build Robot Code". This will build the robot code for the robot, as well as simulation (desktop).

## To run:
- To run in simulation, press Ctrl+Shift+P and search for "Simulate Robot Code". This will launch the simulation.
- To run the code on the robot, press Ctrl+Shift+P and search for "Deploy Robot Code". This will search for a RoboRio over USB or the network and upload the code to the robot.

## Running formatting:
- Run wpiformat with your venv activated

## To make a PR:
- Create a branch from main
- Push to your branch
- Make sure your commits follow the formatting rules or else you can't merge
- Make a PR on github
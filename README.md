# InvestiGator
MIL's UAV Drone for Competition in RobotX.

## Current Status:
InvestiGator is flying!

## Work so Far
The parts for InvestiGator have arrived and have been assembled. The ArduPilot parameters for the GPS and Radio Telemetry have been set. The ESCs have been flashed to the latest firmware and have been set to DShot1200. The RFD900x modems have been configured in a mesh network, and are confirmed to be working together. Test scripts are included in ./Tests for forming a mavlink connection and for sending/receiving serial messages through the radio modems.

## Raodmap and Milestones
The next step for InvestiGator is to be tuned for tighter flight control. This will involve several test flights to determine the necessary gain changes to the PID inputs in each axis of movement. 

Once a successful tuning has been achieved, InvestiGator will be ready for its first autonomous flight through a connection with the Raspberry Pi 5.

In parallel to getting ready for autonomous flight, the detection algorithm for vision control is being written. Once pose estimates works in isolation and InvestiGator is flying stably, the systems will be joined and precision landing will be tested.

## Architecture
The drone consists of two computers, a Raspberry Pi 5 and an Orange Cube+ flight controller. The flight controller receives commands from the Pi for vision related tasks.

The flight controller is connected to a ground station with a radio modem connected to a third computer, the ground station. The ground station sends and receives messages from the drone to start missions, update status, and return points of interest as the drone completes its tasks.

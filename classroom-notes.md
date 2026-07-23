# Sage Summer Camp 2026

## July 21, 2026

### Topics Covered
- Introduction to the Sage platform
- Linux terminal
- Python basics
- Cookiecutter
- Nano editor
- Camera tutorial

### Notes
This file will be updated throughout the Summer Camp with notes, observations, useful Linux commands, and Python examples.
## July 23, 2026

### Project: Wi-Fi HaLow Audio and Video Edge Monitoring

#### Objective

Investigate the feasibility of using a Heltec HT-HC33 edge device equipped with a camera and an SPH0645 I2S MEMS microphone to capture video and audio in the field. The data will be transmitted using Wi-Fi HaLow (IEEE 802.11ah) to a Heltec HT-H7608 Wi-Fi HaLow Gateway, which will forward the data over Ethernet to a Sage Node for edge AI processing, visualization, and storage.

#### Progress

- Selected the Heltec HT-HC33 as the field edge device.
- Selected the SPH0645 I2S MEMS microphone for audio capture.
- Selected the Heltec HT-H7608 as the Wi-Fi HaLow Gateway.
- Designed the initial system architecture using Mermaid AI.
- Refined the architecture by simplifying the communication path:
  - HT-HC33
  - Wi-Fi HaLow (IEEE 802.11ah)
  - HT-H7608 Gateway
  - Ethernet
  - Sage Node
- Removed the optional Wi-Fi HaLow USB Dongle from the current design to simplify the initial architecture.

#### Current Status

The first version of the system architecture diagram has been completed and will continue to evolve as the project progresses.

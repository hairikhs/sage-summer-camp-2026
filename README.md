# SageHaLow: A Wi-Fi HaLow Edge Imaging System for the Sage Platform

**Author:** Hairik Honarchian  
**Institution:** Colorado State University  
**Program:** SAGE Summer Camp 2026

---

# Introduction

Environmental monitoring increasingly relies on distributed edge devices capable of collecting data and performing intelligent processing near the point of measurement. This project investigates the feasibility of combining Wi-Fi HaLow communication with the SAGE Edge Platform to create a long-range, low-power system capable of transmitting images from remote field locations to a SAGE node for future edge AI applications.

The proposed system uses a Heltec HT-HC33 edge device equipped with a camera to capture environmental images. Images are transmitted using Wi-Fi HaLow (IEEE 802.11ah) to a Heltec HT-H7608 Wi-Fi HaLow Gateway, which forwards the data over Ethernet to a SAGE Thor node for processing, visualization, and storage.

---

# Project Goals

The objectives of this project were to:

- Design a complete Wi-Fi HaLow imaging architecture for the SAGE platform.
- Capture environmental images using the Heltec HT-HC33 camera.
- Transmit images over Wi-Fi HaLow (IEEE 802.11ah).
- Forward image data through the Heltec HT-H7608 Gateway.
- Deliver images to a SAGE Thor node through Ethernet.
- Establish a foundation for future edge AI applications.

---

# System Architecture

The proposed communication pathway is:

```
Heltec HT-HC33 Camera
        │
        ▼
Wi-Fi HaLow (IEEE 802.11ah)
        │
        ▼
Heltec HT-H7608 Gateway
        │
        ▼
Ethernet
        │
        ▼
SAGE Thor Node
```

A detailed Mermaid architecture diagram and supporting figures are available in the **ARCHITECTURE** folder.

---

# Current Status

During the SAGE Summer Camp, the following milestones were achieved:

- Designed the complete SageHaLow system architecture.
- Successfully configured the Heltec HT-HC33 camera.
- Successfully configured the Heltec HT-H7608 Wi-Fi HaLow Gateway.
- Successfully demonstrated end-to-end image transmission from the Heltec HT-HC33 through the HT-H7608 Wi-Fi HaLow Gateway to the SAGE Thor node.
- Created a technical project report.
- Developed a conference presentation.
- Organized the project GitHub repository.
- Prepared the project for future expansion to edge AI applications.

---

# Repository Contents

```
ARCHITECTURE/
    Mermaid source files
    Architecture diagrams

PRESENTATION/
    PowerPoint presentation
    PDF presentation

REPORT/
    Technical project report

classroom-notes.md
    Daily Summer Camp notes

README.md
    Project overview
```

---

# Future Development

Planned future work includes:

- Integration of a digital MEMS microphone for synchronized audio capture.
- Event-driven image acquisition.
- On-device edge AI inference.
- Long-term environmental monitoring deployments.
- Integration of additional environmental sensors.

---

# Acknowledgments

This work was completed during the **SAGE Summer Camp 2026**.

The author gratefully acknowledges the guidance and support provided by the SAGE organizers, instructors, mentors, and fellow participants throughout the Summer Camp.

---

# Author

**Hairik Honarchian**

Ph.D. Student  
Department of Soil and Crop Sciences  
Colorado State University

SAGE Summer Camp 2026

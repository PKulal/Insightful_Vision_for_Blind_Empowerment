
<div align="center">
  <h1 align="center">Insightful Vision for Blind Empowerment</h1>  
    <img src="https://media.giphy.com/media/rKkLbWAThKkgg/giphy.gif" alt="Assistive Technology for Visually Impaired" />
    
</div> 
This project introduces a transformative assistive technology designed specifically for visually impaired individuals, enhancing their ability to navigate and interact with their environment safely and independently.

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Requirements](#requirements)
- [Installation](#installation)
- [System Overview](#system-overview)
- [Features](#features)
- [Project Report](#project-report)
- [Authors](#authors)
- [Show your support](#show-your-support)

## Introduction

Our project aims to empower visually impaired individuals by integrating object detection, voice-guided navigation, and essential functionalities to offer unparalleled autonomy and safety. The system employs an ESP32 microcontroller, ultrasonic sensors, and Bluetooth connectivity for seamless navigation and obstacle detection.

## Requirements

- ESP32 Microcontroller
- Ultrasonic Sensors
- IP Camera
- DHT Sensor
- Speaker
- Tactile Switch
- XAMPP for server-side operations
- Mobile application developed using Flutter

## Installation

1. Install XAMPP.
2. Clone the project repository to the root of the XAMPP server.
3. Open the project in XAMPP.
4. Import the SQL file from the `database` folder into the XAMPP server.
5. Open a browser and navigate to [localhost:8080](localhost:8080).
6. Ensure all required hardware components are connected and configured as per the project documentation.
7. Hurray! The project is now running.

## System Overview

The system integrates various technologies to provide a comprehensive navigation assistance solution:

### Hardware Components

- **ESP32 Microcontroller**: Acts as the main controller.
- **Ultrasonic Sensors**: Detects obstacles.
- **IP Camera**: Captures real-time images for object and facial recognition.
- **DHT Sensor**: Monitors environmental conditions.
- **Speaker**: Provides auditory feedback.
- **Tactile Switch**: User input for different functions.

### Software Components

- **Python**: Programming language for developing the core functionalities.
- **OpenCV**: Library for computer vision tasks.
- **TensorFlow Keras**: Used for sentiment analysis.
- **YOLO**: Algorithm for object detection.
- **Arduino**: For programming the ESP microcontroller.
- **Flutter**: For mobile application development.

## Features

### Obstacle Detection

Ultrasonic sensors connected to the ESP32 detect obstacles and provide immediate audio feedback through a speaker.

### Object Recognition

A deep learning model identifies objects in real-time images captured by the camera. Recognized objects are announced to the user via text-to-speech functionality.

### Facial Recognition and Sentiment Analysis

The camera captures facial features, and a trained neural network recognizes faces and analyzes expressions to provide emotional insights.

### Weather Reporting

A DHT sensor monitors environmental conditions and provides verbal updates on the weather, helping users make informed navigation decisions.

### Emergency Assistance

Users can trigger a distress signal via Bluetooth, which communicates with a mobile application and a XAMPP server running dedicated algorithms for object detection and face recognition.

## Project Report

The project report includes:

- **Introduction**
- **Literature Review**
- **Methodology**
- **Results**
- **Conclusion**
- **References**

The report features flow charts, ERDs, and UML diagrams, and is available in Microsoft Word format.

## Project Images
<div align="center">
  <h1 align="center">A Device which Communicate with Blind People</h1>  
    <img src="BlindStick/Img/IMG-20240418-WA0002.jpg" alt="Insightful Vision for Blind Empowerment" />
    
</div>

<div align="center">
  <h1 align="center">Project Structure and Hardware Connections</h1>  
  <div style="display: flex; justify-content: center;">
    <img src="BlindStick/Img/IMG-20240414-WA0002.jpg" alt="Insightful Vision for Blind Empowerment" width="300" style="margin-right: 100px;"/>
    <img src="BlindStick/Img/IMG-20240414-WA0003.jpg" alt="Insightful Vision for Blind Empowerment" width="300" style="margin-left: 100px;"/>
  </div>
</div>



## Authors

- [@PKulal](https://github.com/PKulal)
- [@Praneeth-Jain](https://github.com/Praneeth-Jain/)
- S Harshavardhan
- Thoufeeq M I

## Show your support

If you like this project, please consider giving a star on GitHub and providing feedback to help us improve and expand the system.

---

Feel free to reach out to us for any queries or contributions to this project. Together, we can make navigation safer and more accessible for visually impaired individuals.

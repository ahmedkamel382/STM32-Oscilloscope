# ⚡ Digital Storage Oscilloscope (STM32F401)

![STM32](https://img.shields.io/badge/MCU-STM32F401-blue)
![Python](https://img.shields.io/badge/Python-3.x-yellow)
![Machine Learning](https://img.shields.io/badge/AI-Random_Forest-orange)
![Academic](https://img.shields.io/badge/Course-Embedded_Systems_ECE5206-brightgreen)

A high-performance, low-cost Digital Storage Oscilloscope (DSO) engineered from the component level up. Built around the **STM32F401 "Black Pill"**, this project bridges embedded hardware and modern data science by combining a custom Analog Front-End (AFE), deterministic DMA-paced sampling, high-speed USB-CDC telemetry, and an offline Machine Learning waveform classifier.

This project was developed for the **Embedded Systems (ECE5206)** course at the **Arab Academy for Science, Technology and Maritime Transport (AASTMT)**, supervised by **Dr. Amr Fahmy**.

---

## 🚀 Key Features

* **Bipolar Signal Acquisition:** A custom analog conditioning circuit featuring a 1.65V virtual ground allows the single-supply microcontroller ADC to safely read alternating current (AC) waveforms.
* **Active Hardware Protection:** Bipolar Junction Transistor (BJT) clamping arrays (`2N3904` / `2N3906`) actively protect the STM32 silicon from over-voltage and negative voltage spikes.
* **Deterministic Firmware Architecture:** Hardware Timer 2 (`TIM2`) strictly paces ADC conversions, completely bypassing software polling delays.
* **Zero-Loss Data Capture:** Direct Memory Access (DMA) utilizing a Ping-Pong (Half/Full) buffering strategy ensures continuous, uninterrupted sampling while the CPU processes data.
* **High-Speed USB Streaming:** A custom USB-CDC pipeline streams raw binary data (736-byte packets) to the host PC, vastly outperforming standard UART communication.
* **Edge AI Classification:** A fully offline `scikit-learn` Random Forest machine learning model analyzes live mathematical features to automatically classify incoming signals (`Sine`, `Square`, `Triangle`, `Noise`, or `DC`).
* **Zero-Lag Desktop UI:** A multi-threaded Python companion application built with `PyQtGraph` delivers 60-FPS real-time rendering alongside an embedded SQLite database for waveform logging.

---

## 🧰 Technology Stack

### Hardware

* **Microcontroller:** STM32F401CCU6 (`84 MHz Core`, `48 MHz USB`)
* **Local Display:** 240×240 ST7789 TFT Display (`SPI`)
* **Analog Front-End:** Precision `33kΩ / 10kΩ` attenuation networks with `2N3904/06` hardware clamps
* **PCB Design:** Custom single-sided PCB routed in KiCad with dedicated power-rail decoupling

### Software & Firmware

* **Embedded Development:** C/C++, STM32CubeIDE (`HAL` & `LL Drivers`)
* **Desktop Application:** Python 3, PyQtGraph, PyQt5, SQLite3
* **Machine Learning:** `scikit-learn`, `numpy`, `joblib`

> The AI classifier was trained on approximately **6,000 synthesized waveforms**.

---

## 📊 Hardware Specifications & Limits

| Specification | Value | Notes |
|---|---|---|
| **Max Input Voltage** | ~ `-5.44V` to `+8.74V` | Scaled via a `0.1315` attenuation factor |
| **Max Sampling Rate** | ~ `1.56 Msps` | ADCCLK pushed to `42 MHz (DIV2)` |
| **Practical Bandwidth** | ~ `150 kHz` | Based on requiring 10 data points per wave cycle |
| **Hardware Filter** | RC Low-Pass | V1.0 uses a `33pF` capacitor (`102 MHz` cutoff). V2.0 recommends a `4.7nF` capacitor to align the anti-alias cutoff to approximately `720 kHz`. |

---

## ⚙️ Getting Started

### 1. Hardware Setup

1. Assemble the Analog Front-End using the provided KiCad schematics located in:

```text
/Hardware Design
```

2. Ensure proper grounding between the device under test (for example, an ICL8038 signal generator) and the STM32.

3. Connect the STM32 board to your PC using a USB Type-C cable.

---

### 2. Firmware Flashing

1. Open the project using **STM32CubeIDE**.
2. Build the firmware.
3. Flash the firmware to the STM32F401 using an **ST-Link V2** programmer.

---

### 3. Running the Desktop Application

Ensure Python 3 is installed on your system.

Navigate to the desktop application directory and install the dependencies:

```bash
cd dso_pc
pip install -r requirements.txt
```

Launch the desktop UI:

```bash
python dso_app.py
```

> The AI classifier (`dso_model.joblib`) is pre-trained.
>
> If you wish to retrain the model using different parameters, execute:
>
> ```bash
> python train_model.py
> ```
>
> before launching the application.

---

## 👨‍💻 Project Team

### Ahmed Khalid Mohamed

* Lead Firmware Engineer & System Architect
* ADC/DMA Pipeline
* USB-CDC Telemetry
* Clock Tree & Peripheral Configurations

### Adel Alaa

* Lead PCB Designer & Hardware Integration
* KiCad PCB Routing
* Power Rail Decoupling
* Physical Hardware Assembly

### Bassel Adel

* Lead Analog Hardware Engineer
* Analog Front-End (AFE) Prototyping
* Virtual Ground Circuit Design
* Transistor Clamping Architecture
* Signal Testing Bench

### Omar Abdel Nasser Omar

* Lead Software & Machine Learning Engineer
* PyQtGraph Desktop UI
* Multithreaded Data Rendering
* Random Forest AI Training Pipeline

---

## 📝 License & Academic Disclaimer

This repository contains academic coursework and is provided **as-is** for educational purposes.

Do **not** connect this device directly to mains AC voltage or high-voltage circuits without proper industrial isolation and safety equipment.
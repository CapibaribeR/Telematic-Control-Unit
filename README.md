# Telemetry Control Unit (TCU) Overview

Our Telemetry Control Unit (TCU) is responsible for collecting, processing, and storing critical data from our Formula Student Electric vehicle, while also providing real-time information to the driver and enabling GPS-based tracking.

We are currently using an **ESP32** microcontroller as the main processing unit, chosen for its dual-core architecture and integrated wireless capabilities. The firmware is built on **FreeRTOS**, allowing efficient multitasking and dedicated core allocation for communication, data logging, and display updates.

## Data Acquisition
- The TCU receives data from both the **Battery Management System (BMS)** and the **Vehicle Control Unit (VCU)** via **CAN bus**, using the native **TWAI driver** for robust and low-latency communication.
- BMS data includes voltage, current, state of charge, and temperatures.
- VCU data includes motor RPM, torque, temperatures, and other powertrain metrics.
- A **NEO-6M GPS module** is integrated for vehicle tracking and location-based analysis.

## Storage & Logging
- All data is logged to an **SD card** in CSV format for post-run analysis.
- A **Built-in timer** provides timestamps for accurate event correlation.
- The system is designed to continue operating even if the SD card is missing, ensuring uninterrupted telemetry.

## Display & Monitoring
- A **VIctor Vision 4-inch serial display** presents key metrics such as vehicle speed, SOC and temperatures in real time.
- A **1.3-inch OLED display** present status from CANBUS, GPS and SD card.

## System Architecture
- **Core 0** handles CAN communication, GPS data acquisition and writes to the SD card.
- **Core 1** processes incoming data and updates display. 
- FreeRTOS tasks are prioritized to avoid blocking and maintain consistent sampling rates.

## Future Enhancements
- Integration of a **LoRa module** for real-time wireless telemetry to the pit.
- Advanced display interface with graphical elements and event alerts.

## Summary
The TCU serves as the data hub of the vehicle, ensuring that all relevant performance and health metrics are collected, displayed, and stored with high reliability. With its multitasking architecture and robust communication design, it plays a key role in both driver feedback and post-race analysis.

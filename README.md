Biodiversity Sensor Project
===========================

Overview
--------

This repository contains two distinct biodiversity sensors, both optimized for ultra-low power consumption and designed for remote monitoring in natural environments.
The sensors focus on capturing audio and visual data to help monitor wildlife, particularly bird species, through sound and image detection.
The data is stored locally and uploaded to AWS for analysis when internet connectivity is available.

The project utilizes custom hardware based on the **nRF5340DK** development board, integrated with GSM modules for remote data transmission.

Sensors
-------

Audio Sensor (Bird-Frequency Sound Detector)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This sensor records surrounding sound and filters bird-specific frequencies. It is ideal for long-term field deployments due to its efficient power consumption and local storage capability. The audio recordings are later uploaded to AWS for further analysis.

Vision Sensor (Image Capture Device)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This sensor captures environmental images at regular intervals using an **ArduCAM** camera. It is also designed for ultra-low-power operation, suitable for extended remote monitoring. Images are stored locally and transmitted via a GSM module when network access is available.

Key Features
------------

Audio Sensor
~~~~~~~~~~~~
- **Bird-Frequency Sound Detection**: Filters and records only bird-related sounds using a microphone.
- **6-Second Recordings**: Each recording is 6 seconds long, stored on an SD card.
- **Up to 300,000 Files**: The device can store up to 300,000 recordings.
- **AWS Integration**: Automatically uploads data to AWS when connected to the internet.
- **Ultra-Low Power Consumption**: Consumes only 0.2 Watts, ideal for energy-sensitive deployments.

Vision Sensor
~~~~~~~~~~~~~
- **ArduCAM 3MP 3.3V SPI Camera**: Captures high-quality images every 20 seconds.
- **Efficient Data Storage**: Images are stored locally and transmitted when a network is available.
- **Remote Data Upload**: Captured images are uploaded via GSM to the cloud or server for analysis.
- **Ultra-Low Power Operation**: Consumes only 1 Watt for extended battery life.

Hardware Specifications
------------------------

- **Custom nRF5340DK Board**: Central control for both sensors, handling data processing and communication.
- **GSM Modules**:
  - **EG25** for the audio sensor.
  - **EC200** for the vision sensor.
- **Microphone**: Used by the audio sensor to capture surrounding bird sounds.
- **ArduCAM 3MP Camera**: Used by the vision sensor for capturing images.
- **SD Card**: Used by the audio sensor to store recordings (up to 300,000 files).
- **Power Consumption**:
  - 0.2 Watts for the audio sensor.
  - 1 Watt for the vision sensor.

How It Works
------------

Audio Sensor
~~~~~~~~~~~~
1. **Recording**: The microphone captures ambient sounds, filtering bird-specific frequencies. Each recording lasts for 6 seconds.
2. **Data Backup**: The device saves the recordings on an SD card, capable of storing up to 300,000 files.
3. **AWS Upload**: When internet access is available, the device uploads the recordings to AWS via the GSM EG25 module.
4. **Low Power Operation**: Consumes only 0.2 Watts, making it suitable for long-term deployments.

Vision Sensor
~~~~~~~~~~~~~
1. **Image Capture**: The camera captures an image every 20 seconds.
2. **Data Storage**: Images are saved locally on the device.
3. **Remote Upload**: When the GSM EC200 module connects to the internet, stored images are uploaded to the cloud.
4. **Low Power Mode**: The device operates on 1 Watt of power, ensuring energy-efficient long-term operation.

Installation
------------

1. Clone this repository:
   .. code-block:: bash

      git clone https://github.com/SumanKumar891/Biodiversity_Sensor.git

2. Build and flash the firmware onto the nRF5340DK board:
   .. code-block:: bash

      west build -b nrf5340dk_nrf5340_cpuapp
      west flash

3. Configure the **GSM EG25** module for internet access as described in the `GSM_Configuration.md` file.

4. Set up your AWS IoT credentials and configure the device for AWS communication using the provided certificates.

Usage
-----

Audio Sensor
~~~~~~~~~~~~
Once powered on, the device will:
1. Automatically record bird-frequency audio.
2. Store recordings on the SD card.
3. Upload recordings to AWS when internet access is available.
4. Enter low-power mode between recordings.

Vision Sensor
~~~~~~~~~~~~~
When running, the device will:
1. Capture images at 20-second intervals.
2. Store images locally.
3. Upload images when internet access becomes available.
4. Operate in a low-power state to extend battery life.

Contributing
------------

Feel free to submit issues or pull requests to improve the functionality or performance of either sensor.

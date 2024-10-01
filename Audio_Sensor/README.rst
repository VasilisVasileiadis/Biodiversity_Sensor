Biodiversity Sensor Project
===========================

Overview
--------
This project focuses on monitoring and recording surrounding sound, specifically targeting bird frequencies. The system utilizes a custom nRF5340DK board with an integrated GSM EG25 module.
The device captures 6-second audio recordings and stores them on an SD card. Once internet access becomes available, the recorded files are uploaded to AWS for further analysis and storage.

The device is designed for ultra-low power consumption, making it ideal for long-term, remote deployments. It consumes only 0.2 Watts of power, ensuring minimal energy use while monitoring and recording in the field.

Features
--------
- **Bird-Frequency Sound Detection**: The sensor filters and records only bird frequency sounds from the environment using a microphone.
- **Efficient File Storage**: The device can back up up to 300,000 audio recordings, each with a length of 6 seconds, on an SD card.
- **AWS Integration**: When internet connectivity is available via the GSM module, the stored audio files are uploaded to AWS for cloud-based analysis.
- **Ultra-Low Power Consumption**: Operates on just 0.2 Watts, making it ideal for energy-sensitive deployments.

Hardware Specifications
------------------------
- **Custom nRF5340DK Board**: Central control unit for sensor operation, data processing, and communication.
- **GSM Module (EG25)**: Facilitates internet connectivity for remote file uploads.
- **Microphone**: Captures ambient sounds, specifically filtering for bird frequencies.
- **SD Card**: Storage for up to 300,000 audio files (6 seconds each).
- **Power Consumption**: 0.2 Watts for long-term energy efficiency.

How It Works
------------
1. **Recording**: The microphone captures ambient sound, filtering for bird frequencies. Each recording lasts for 6 seconds.
2. **Data Backup**: The device saves each 6-second recording to an SD card, which can store up to 300,000 files.
3. **AWS Upload**: When the device detects internet access via the GSM EG25 module, it automatically uploads the saved files to AWS.
4. **Low Power Mode**: The system operates with minimal power consumption (0.2 Watts), maximizing battery life for extended field deployments.

Installation
------------
1. Clone this repository.
   .. code-block:: bash

      git clone https://github.com/SumanKumar891/Biodiversity_Sensor.git

2. Follow the instructions in the `firmware` directory to build and flash the firmware onto the nRF5340DK board.

3. Configure the GSM EG25 module for internet access as described in the `GSM_Configuration.md` file.

4. Set up your AWS IoT credentials and configure the device for AWS communication using the provided certificates and instructions.


Contributing
------------
Feel free to submit issues or pull requests. Contributions are welcome to improve the functionality and performance of the project.
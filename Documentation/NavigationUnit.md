# Gravitum Navigation Unit (NU)

# Component:
STM32F405RGTX

## Overview

The Gravitum Navigation Unit (NU) is responsible for providing accurate position, velocity, and attitude data to the Flight Control Unit (FCU). It interfaces with various sensors and peripherals to gather necessary navigation information.

## Key Responsibilities

- Acquiring data from GNSS, IMU, Barometer, Magnetometer, and other sensors.
- Processing sensor data to compute accurate navigation solutions.
- Communicating navigation data to the FCU for flight control decisions.

## Interfaces

- **Flight Control Unit**: Sends processed navigation data including position, velocity, and attitude.
- **Sensors**: Interfaces with GNSS, IMU, Barometer, Magnetometer, and other navigation-related sensors.
- **Memories**: Stores calibration data, and configurations for navigation algorithms.

## Communication Protocols

- **UART1**: Primary communication channel with the Flight Control Unit.
- **UART2**: Firmware debugging and diagnostics.
- **UART3**: Communication with GNSS module.
- **UART4**: Extra UART for additional peripherals, a.g., Dynamic pressure sensor (Pitot tube).
- **I2C1**: Secondary communication with the Flight Control Unit.
- **I2C2**: Communication with peripherals such as Temperature sensor, Barometer, EEPROM, and Magnetometer.
- **I2C3**: Extra I2C for additional peripherals, e.g., Pitot sensor.
- **SPI1**: Interface with flash memory for data storage, with one GPIO chip select.
- **SPI2**: Interface with IMU for high-speed data acquisition, with one GPIO chip select.
- **SWD**: Used for coding and debugging the NU firmware.
- **GPIOs**: Various GPIO pins for input and output communication with the Flight Control Unit and sensors.
- **DFU_USB**: USB interface for Device Firmware Upgrade (DFU) operations.

## Explaining each area of hardware schematic related to NU

- **Decoupling Capacitors**: Ensure stable power supply to the NU by filtering out noise and voltage spikes.
- **High-Speed Crystal Oscillator**: Provides a precise clock signal for the microcontroller to ensure accurate timing and operation.
- **VDDA Filter**: Filters the analog power supply to reduce noise for sensitive analog components.
- **Boot Mode**: Configures the microcontroller's startup behavior, allowing selection between normal operation and firmware update modes.
- **Reset Push Button**: Allows manual resetting of the NU for troubleshooting or firmware updates.
- **I2C Test Points**: Provide access for testing and debugging I2C communication lines.
- **Indicators (LEDs)**: Visual indicators for power status, communication activity, and error states.
- **I2C Pull-up Resistors**: Ensure proper voltage levels on I2C lines for reliable communication.
- **Global Navigation Satellite System (GNSS)**: Provides satellite-based positioning data to the NU.
- **Temperature Sensor**: Measures ambient temperature for environmental compensation in navigation calculations.
- **Barometer**: Measures atmospheric pressure to assist in altitude determination.
- **Magnetometer**: Measures magnetic field strength to determine heading information.
- **Inertial Measurement Unit (IMU)**: Provides acceleration and angular rate data for attitude and motion sensing.
- **Flash Memory**: Non-volatile storage for calibration data, configurations, and logs.
- **Extra Connectors**: Provide physical interfaces for connecting additional sensors or peripherals as needed.
- **EEPROM**: Non-volatile memory for storing small amounts of data that must be saved when power is removed.
- **FCU - NU (GPIOs)**: General-purpose input/output pins used for communication and control signals between the NU and FCU.
- **FCU - NU (I2C)**: I2C communication lines dedicated to data exchange between the NU and FCU.
- **FCU - NU (UART)**: UART communication lines dedicated to data exchange between the NU and FCU.
- **GPS Backup Battery**: Ensures that the GNSS module retains its data when the main power is off, allowing for faster satellite acquisition upon power-up.
- **DFU USB Connectors**: Provide the physical interface for USB-based firmware updates and diagnostics.
- **Logging Debug USB**: USB interface dedicated to logging and debugging purposes during development and testing.
- **Serial Wire Debug Connectors**: Provide access for debugging and programming the NU firmware via SWD protocol.

## Flight Modes

The NU supports multiple flight modes, including:

- **Normal Mode**: Standard operation for navigation data acquisition and processing.
- **Calibration Mode**: Used for calibrating sensors such as IMU and Magnetometer.
- **Diagnostic Mode**: Allows for testing and troubleshooting of navigation systems and sensors.
- **Low Power Mode**: Reduces power consumption during periods of inactivity or when high-frequency data is not required.

## Indicators LEDs

The Gravitum Navigation Unit features several indicator LEDs to provide visual feedback on its operational status:

| LED Color | Function                      | Description                                      |
|-----------|-------------------------------|--------------------------------------------------|
| Green     | Power Status                  | Indicates that the NU is powered on and functioning correctly. |
| Blue      | GNSS Lock                     | Indicates successful acquisition of GNSS satellite signals. |
| Yellow    | Data Transmission             | Indicates active data transmission to the FCU.   |
| Red       | Error Indicator               | Indicates an error or fault condition in the NU.  |

## Connectors

TODO
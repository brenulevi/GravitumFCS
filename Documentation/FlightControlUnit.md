# Gravitum Flight Control Unit (FCU)

### Component:
STM32F405RGTX

## Overview

The Gravitum Flight Control Unit (FCU) is the central component responsible for managing and controlling the flight operations of the airplane. It interfaces with Navigation Unit, Memories, Actuators, and Communication systems to ensure safe and efficient flight.

## Key Responsibilities

- Monitoring the health of all subsystems, including power, sensors, actuators, and communication links.
- Managing flight modes and transitions between them.
- Executing control algorithms to maintain stable flight and respond to pilot commands.
- Logging flight data for post-flight analysis and debugging.
- Handling fail-safe mechanisms in case of system failures or unexpected conditions.

## Interfaces

- **Navigation Unit**: Receives position, velocity, and attitude data to make informed control decisions.
- **Memories**: Stores flight parameters, configurations, and logs for retrieval during flight and post-flight analysis.
- **Actuators**: Sends control signals to servos, motors, and other actuators to execute flight maneuvers.
- **Communication Systems**: Exchanges data with ground control to control airplane

## Flight Modes

The FCU supports multiple flight modes, including:

- **Manual Mode**: Direct control by the pilot.
- **Stabilized Mode**: Automatic stabilization while allowing pilot input.
- **Autonomous Mode**: Fully automated flight based on pre-defined waypoints and mission parameters.
- **Return to Home**: Automatically returns the airplane to a predefined home location in case of signal loss or low battery.

## Communication Protocols

- **UART1**: Primary communication channel with the Navigation Unit.
- **UART2**: Firmware debugging and diagnostics.
- **UART3**: Communication with ground control radio.
- **UART4**: Interface with camera to take pictures during flight.
- **I2C1**: Secondary communication with the Navigation Unit.
- **I2C2**: Communication with peripherals such as PWM drivers, EEPROM, and power monitoring.
- **SPI1**: Interface with flash memory for data storage.
- **SWD**: Used for coding and debugging the FCU firmware.
- **SDIO**: Interface with SD card for additional data storage.
- **GPIOs**: Various GPIO pins for input and output communication with the Navigation Unit.
- **DFU_USB**: USB interface for Device Firmware Upgrade (DFU) operations.
- **TIM3**: Controls actuators such as ailerons, elevator, rudder, and flaps.
- **TIM1**: Manages propulsion systems including ESCs for engines.

## Explaining each area of hardware schematic related to FCU

- **Decoupling Capacitors**: Ensure stable power supply to the FCU by filtering out noise and voltage spikes.
- **High-Speed Crystal Oscillator**: Provides a precise clock signal for the microcontroller to ensure accurate timing and operation.
- **VDDA Filter**: Filters the analog power supply to reduce noise for sensitive analog components.
- **Boot Mode**: Configures the microcontroller's startup behavior, allowing selection between normal operation and firmware update modes.
- **Reset Push Button**: Allows manual resetting of the FCU for troubleshooting or firmware updates.
- **I2C Test Points**: Provide access for testing and debugging I2C communication lines.
- **Indicators (LEDs)**: Visual indicators for power status, communication activity, and error states.
- **I2C Pull-up Resistors**: Ensure proper voltage levels on I2C lines for reliable communication.
- **Radio/Camera Connectors**: Provide physical interfaces for connecting external communication and camera modules.
- **Actuators Connections**: Dedicated pins for controlling servos and motors that manage flight surfaces and propulsion.
- **PWM Driver**: Manages pulse-width modulation signals for precise control of actuators or extra peripherals, such as servos, and lighting.
- **Flash Memory**: Non-volatile storage for flight data, configurations, and logs.
- **SD Card**: Additional storage option for extensive data logging and retrieval.
- **EEPROM**: Stores critical configuration data that must persist across power cycles.
- **FCU - NU (GPIOs)**: General-purpose input/output pins for communication and control signals between the FCU and Navigation Unit.
- **FCU - NU (UART1)**: Serial communication channels for data exchange between the FCU and Navigation Unit.
- **FCU - NU (I2C1)**: I2C communication lines for interfacing with the Navigation Unit.
- **DFU USB Interface**: USB connection for firmware updates and diagnostics.
- **Logging Debug USB/UART**: Dedicated interface for debugging and logging during development and testing.
- **Serial Wire Debug (SWD)**: Interface for programming and debugging the FCU firmware.

## Indicator LEDs

The Gravitum FCU includes 4 indicator LEDs to provide visual feedback on the system's status, presented in the following table:

| LED Color | Function                     | Description                                      |
|-----------|------------------------------|--------------------------------------------------|
| Green     | Power Indicator              | Indicates that the FCU is powered on.            |
| Blue      | Communication Status         | Indicates active communication with NU.          |
| Yellow    | Error Indicator              | Flashes to indicate system errors or faults.     |
| Red       | Flight Mode Indicator        | Indicates the current flight mode of the FCU.    |
These LEDs help users quickly assess the operational status of the FCU during flight and troubleshooting.

## Connectors

TODO: Add connector pinouts and descriptions for FCU connections to peripherals and other systems.




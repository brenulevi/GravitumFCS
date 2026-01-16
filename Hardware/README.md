# FCU
- UART1: Comm with NU
- UART2: Logging Debug
- UART3: Extra UART (Radio Control)
- UART4: Extra UART (Camera)

- I2C1: Comm with NU
- I2C2: Peripherals (PWM Driver, EEPROM)

- TIM3: Actuators (CH1 Ailerons, CH2 Elevator, CH3 Rudder, CH4 Flaps)
- TIM1: Propulsions (CH2 ESC1, CH3 ESC2)

- SPI1: Peripherals (Flash)

- SWD: Coding/Debug

- SDIO: SD Card (1bit)

- GPIOA, GPIOC, GPIOE: Comm with NU (Output)
- GPIOB, GPIOD, GPIOF: Comm with NU (Input)

# NU
- UART1: Comm with FCU
- UART2: Logging Debug
- UART3: Peripheral (GNSS)
- UART4: Extra UART

- I2C1: Comm with FCU
- I2C2: Peripherals (Temperature, Baro, EEPROM, MAG)
- I2C3: Extra I2C (Pitot Sensor)

- SPI1: Peripherals (Flash) + 1 GPIO CS
- SPI2: Peripherals (IMU) + 1 GPIO CS

- SWD: Coding/Debug

- GPIO_GPS_PSS
- GPIO_GPS_LNA_EN
- GPIO_GPS_NRST

- GPIOA, GPIOC, GPIOE: Comm with NU (Input)
- GPIOB, GPIOD, GPIOF: Comm with NU (Output)
# ESP32 IMU FIFO Data Acquisition using MPU9250 (SPI)

This project implements high-rate IMU data acquisition on the ESP32 using the MPU-9250 sensor over SPI.
The firmware reads accelerometer, gyroscope, and temperature data from the sensor at 200 Hz, stores the samples in a ring buffer, and periodically logs scaled physical values over UART.

### The project demonstrates:

- SPI communication with a MEMS IMU
- Real-time sensor sampling using esp_timer
- Efficient data buffering using a lock-free ring buffer
- Conversion of raw sensor readings to physical units

### Hardware components

- ESP32 Development Board
- MPU-9250 9-axis IMU sensor module
- Jumper wires
<br>

### SPI Connections

| ESP32 Pin | MPU9250 Pin |
|----------|-------------|
| GPIO23 | MOSI |
| GPIO19 | MISO |
| GPIO18 | SCLK |
| GPIO5  | CS |
| 3.3V   | VCC |
| GND    | GND |
<br>

#### SPI frequency used: 1 MHz

### Sensor Configuration

The MPU-9250 is initialized with the following configuration:

| Parameter | Value |
|----------|-------|
| Clock Source | PLL |
| Sample Rate | 200 Hz |
| Gyro Range | ±500 °/s |
| Accel Range | ±4 g |
| Gyro DLPF | 184 Hz |
| Accel DLPF | 218 Hz |

### Sample rate calculation:

ODR = 1000 Hz / (1 + SMPLRT_DIV)
ODR = 1000 / (1 + 4) = 200 Hz
<br>
Data Acquisition: Every 5 ms (200 Hz) the firmware reads 14 bytes starting from register ACCEL_XOUT_H.

### Data Layout

| Bytes | Data |
|------|------|
| 0–1 | Accel X |
| 2–3 | Accel Y |
| 4–5 | Accel Z |
| 6–7 | Temperature |
| 8–9 | Gyro X |
| 10–11 | Gyro Y |
| 12–13 | Gyro Z |

All values are 16-bit signed integers (big-endian).
<br> 
### Ring Buffer
To avoid data loss, samples are stored in a circular buffer.

Buffer size = 2048 samples
Sampling rate = 200 Hz
Buffer duration ≈ 10 seconds
<br> 
Sensor Scaling: Raw data is converted to physical units using the MPU-9250 sensitivity constants.

<br>

From the MPU-9250 datasheet:
<br> 
### Accelerometer
±4g range

Sensitivity = 8192 LSB/g

accel_g = raw / 8192

<br> 

### Gyroscope
±500 °/s range

Sensitivity = 65.5 LSB/(°/s)

gyro_dps = raw / 65.5

<br>

### Temperature 

Temp(°C) = (raw / 333.87) + 21

Reading 14 bytes(IMU + temp) instead of 12 is convenient because the IMU registers are contiguous

<br>


This project reads **accelerometer and gyroscope data from an IMU (MPU9250 / MPU6050)** using an **ESP32** and streams the data as **CSV over serial**.

The project uses **PlatformIO in VSCode with ESP-IDF**.


### Build

Compile the project:

```
pio run
```


### Upload Firmware

Flash the program to the ESP32:

```
pio run --target upload
```


### View Live IMU Data

Open the serial monitor:

```
pio device monitor
```

### Example Output

```
I (275) main task: Calling app_main()
I (275) IMU: === MPU-9250 SPI Acquisition starting ===
I (635) IMU: MPU-9250 detected  WHO_AM_I=0x70 ✓
I (635) IMU: Sampling at 200 Hz | Ring buffer: 2048 samples = 10.2 s
I (635) main_task: Returned from app_main()

I (1635) IMU: [1373562 us] total=199 (+199/s) buf=199 overruns=0 | A(g): 0.007 -0.001 1.004  G(dps): 1.69 1.04 -1.40  T: 33.7°C
I (2635) IMU: [2373562 us] total=399 (+200/s) buf=399 overruns=0 | A(g): 0.009 -0.003 1.007  G(dps): 1.62 0.92 -1.48  T: 33.7°C
I (3635) IMU: [3373562 us] total=599 (+200/s) buf=599 overruns=0 | A(g): 0.006 0.001 1.006  G(dps): 1.53 1.01 -1.36  T: 33.7°C
I (4635) IMU: [4373562 us] total=799 (+200/s) buf=799 overruns=0 | A(g): 0.005 -0.003 1.007  G(dps): 1.54 0.90 -1.39  T: 33.7°C
I (5635) IMU: [5373562 us] total=999 (+200/s) buf=999 overruns=0 | A(g): 0.007 -0.003 1.009  G(dps): 1.53 1.01 -1.40  T: 33.7°C
I (6635) IMU: [6373562 us] total=1199 (+200/s) buf=1199 overruns=0 | A(g): 0.006 0.000 1.011  G(dps): 1.56 0.75 -1.30  T: 33.7°C
I (7635) IMU: [7373562 us] total=1399 (+200/s) buf=1399 overruns=0 | A(g): 0.011 -0.004 1.005  G(dps): 1.50 0.99 -1.37  T: 33.7°C
I (8635) IMU: [8373562 us] total=1599 (+200/s) buf=1599 overruns=0 | A(g): 0.010 -0.004 1.012  G(dps): 1.57 0.85 -1.36  T: 33.7°C
I (9635) IMU: [9373562 us] total=1799 (+200/s) buf=1799 overruns=0 | A(g): 0.006 -0.008 1.010  G(dps): 1.47 1.01 -1.33  T: 33.7°C
I (10635) IMU: [10373562 us] total=1999 (+200/s) buf=1999 overruns=0 | A(g): 0.010 -0.005 1.002  G(dps): 1.44 1.01 -1.44  T: 33.8°C
```
### Log Data to CSV

Save the IMU data directly to a file:

```
pio device monitor --raw > imu_log.csv
```

Stop logging with:

```
CTRL + C
```


### Reference Documents

ESP32 Datasheet

MPU9250 Datasheet

MPU9250 Register Map

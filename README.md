# L.I.N.K. ESP32 Firmware

ESP32-S3 firmware for the binocular HUD. Manages sensors, computes waypoints, and streams data to the phone app over BLE.

## Architecture

```
┌───────────────────────────────────────────────────────────┐
│                     ESP32-S3 Main Loop                     │
│                                                           │
│  ┌─────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────┐  │
│  │ IMU     │ │ GNSS     │ │ Baro     │ │ LiDAR        │  │
│  │ BNO055  │ │ NEO-M8N  │ │ BMP280   │ │ TFMini-Plus  │  │
│  │ (I²C)   │ │ (UART2)  │ │ (I²C)    │ │ (UART1)      │  │
│  └────┬────┘ └────┬─────┘ └────┬─────┘ └──────┬───────┘  │
│       └───────────┴────────────┴───────────────┘          │
│                        │                                  │
│              TelemetrySnapshot (20 Hz poll)                │
│                        │                                  │
│              ┌─────────┴─────────┐                        │
│              │ Telemetry JSON    │ ← 5 Hz BLE notify      │
│              │ Pin Payload JSON  │ ← on button press      │
│              └─────────┬─────────┘                        │
│                        │                                  │
│              ┌─────────┴─────────┐                        │
│              │  BLE GATT Server  │                        │
│              │  (3 characteristics)                       │
│              └─────────┬─────────┘                        │
│                        │                                  │
│              ┌─────────┴─────────┐                        │
│              │   Pin Queue       │ ← retry until ACK      │
│              └───────────────────┘                        │
└───────────────────────────────────────────────────────────┘
                         │ BLE
                         ▼
                ┌─────────────────┐
                │  Phone PWA      │
                │  (hud-app)      │
                └─────────────────┘
```

## BLE GATT Service

| UUID                                     | Type   | Description                |
|------------------------------------------|--------|----------------------------|
| `4c494e4b-4855-4400-b000-000000000000`   | Service | LINK-HUD                  |
| `4c494e4b-4855-4400-b000-000000000001`   | Notify  | Telemetry stream (5 Hz)   |
| `4c494e4b-4855-4400-b000-000000000002`   | Notify  | Pin payload (on ping)     |
| `4c494e4b-4855-4400-b000-000000000003`   | Write   | ACK from phone            |

## Boot Sequence

1. Init serial + GPIO
2. Init sensor drivers (IMU, GNSS, Baro, LiDAR)
3. Run self-tests on each module
4. Degrade gracefully if a module fails (disable it, warn user)
5. Start BLE GATT advertising
6. Enter main loop

## Pin Configuration

Edit `include/config.h` to match your wiring:

| Signal    | Default GPIO | Notes                    |
|-----------|-------------|--------------------------|
| I²C SDA   | 21          | IMU + Baro               |
| I²C SCL   | 22          | IMU + Baro               |
| GNSS RX   | 16          | ESP RX ← GPS TX          |
| GNSS TX   | 17          | ESP TX → GPS RX          |
| LiDAR RX  | 26          | ESP RX ← LiDAR TX        |
| LiDAR TX  | 27          | ESP TX → LiDAR RX        |
| Ping BTN  | 0           | Active LOW (BOOT button) |
| Batt ADC  | 34          | Voltage divider          |

## Building

Requires [PlatformIO](https://platformio.org/):

```bash
# Build
pio run

# Upload to ESP32
pio run -t upload

# Serial monitor
pio device monitor -b 115200
```

## File Structure

```
esp32-firmware/
├── platformio.ini              # PlatformIO config
├── include/
│   ├── config.h                # GPIO pins, timing, UUIDs
│   └── sensor_types.h          # Shared data structures
├── src/
│   ├── main.cpp                # Boot + main loop
│   ├── ble_server.h/.cpp       # BLE GATT server
│   ├── telemetry_json.h/.cpp   # JSON serialisation
│   ├── waypoint.h              # Destination point math
│   ├── pin_queue.h/.cpp        # Local retry queue
│   └── drivers/
│       ├── imu_driver.h/.cpp   # BNO055
│       ├── gnss_driver.h/.cpp  # NEO-M8N / TinyGPS++
│       ├── baro_driver.h/.cpp  # BMP280
│       └── lidar_driver.h/.cpp # TFMini-Plus
└── README.md
```

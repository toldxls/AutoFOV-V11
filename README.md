# AutoFOV V11 (WiFi & BLE Edition)

AutoFOV is a specialized microscopy field-of-view (FOV) calculator, automated focus stacking assistant, and wireless camera trigger. Powered by an ESP32-S3, it uses a Time-of-Flight (ToF) sensor to measure subject distance and calculates the real-time FOV based on a calibrated linear regression model. 

This version (V11 patched3) introduces concurrent Bluetooth Low Energy (BLE) HID triggering and a high-speed WiFi WebSocket server for remote telemetry and control.

## Features

* **Real-Time FOV Calculation:** Uses a VL53L4CX ToF sensor to measure distance and computes FOV for 5x, 10x, and 20x objectives.
* **Custom Calibration:** Calibrate the device to your specific optical setup using a physical demarcation standard. Calculates standard error (RMSE) and R² of the calibration fit.
* **Stack Calculator:** Automatically calculates step size, total depth, and required images for focus stacking based on objective numerical aperture (NA) and depth of field (DOF).
* **BLE Camera Triggering:** Acts as a BLE HID Keyboard. Sends an F12 keystroke to your connected PC or camera rig when the hardware trigger is activated.
* **Web Dashboard & Telemetry:** Connects to your local WiFi to provide a high-speed (~30Hz) WebSocket telemetry stream (distance, FOV, signal rate) and remote control over settings.
* **Captive Portal Setup:** First-time WiFi setup is handled via a built-in access point (AutoFOV-Setup).
* **PSRAM-Backed UI:** Fluid, flicker-free UI utilizing off-screen 16-bit sprites buffered in PSRAM.
* **Customization:** Adjustable screen brightness, sleep timeouts, and multiple color themes (Classic, Midnight, Forest, Daylight) with adjustable tint.

## Hardware Requirements

* **Microcontroller:** ESP32-S3 (Requires 2MB+ PSRAM and 4MB+ Flash)
* **Display:** 2.8" ILI9341 TFT SPI Display
* **Touch Controller:** FT6206 Capacitive Touch (I2C)
* **Sensor:** VL53L4CX Time-of-Flight Sensor (I2C)
* **Trigger Interface:** Optocoupler or physical switch

### Pin Configuration

| Component | Pin (ESP32-S3) | Notes |
| :--- | :--- | :--- |
| **TFT CS** | 9 | SPI Chip Select |
| **TFT DC** | 10 | SPI Data/Command |
| **TFT RST** | 6 | SPI Reset |
| **TFT Backlight** | A2 (LITE_PIN) | PWM Brightness Control |
| **Touch Int** | 11 (TOUCH_INT_PIN) | Active Low Interrupt |
| **Camera Trigger** | A4 | Input Pullup |
| **Trigger LED** | A3 | Active-Low PWM output (3.3V → LED → A3) |
| **I2C SDA / SCL** | Default | Shared by FT6206 and VL53L4CX |

## Software Dependencies

Install the following libraries via the Arduino Library Manager or manually:
* `Adafruit_GFX`
* `Adafruit_ILI9341`
* `Adafruit_FT6206`
* `vl53l4cx_class` (STM32duino)
* `NimBLE-Arduino`
* `ESPAsyncWebServer` (by me-no-dev)
* `AsyncTCP` (by me-no-dev)
* `ArduinoJson` (v6.x)

## Installation & Flashing

### 1. Arduino IDE Setup
Because this firmware heavily relies on PSRAM for UI buffering and features concurrent radio usage, strict build settings are required. Configure your board settings as follows:
* **Board:** ESP32S3 Dev Module
* **PSRAM:** QSPI PSRAM 
* **Partition Scheme:** Huge APP 
* **Events Run On:** Core 1
* **Arduino Runs On:** Core 1

### 2. Upload the Web Interface (LittleFS)
The web interface HTML is required for the WiFi dashboard to load.
1. Create a `data` folder in the same directory as your `.ino` files.
2. Place your `index.html` file inside the `data` folder.
3. Use the Arduino ESP32 LittleFS Data Upload Tool to flash the contents of the `data` folder to the ESP32.

### 3. Flash the Firmware
Compile and upload the combined sketch (`AutoFOV_V11_patched3.ino` and `AutoFOV_V11_patched3_wifi.ino`).

## Usage Instructions

### WiFi & Web Interface
1. **Initial Boot:** If no WiFi credentials are saved, the device will boot into Captive Portal mode.
2. **Connect to AP:** Connect your phone or PC to the WiFi network `AutoFOV-Setup`.
3. **Configure:** A setup page will pop up automatically. Enter your home WiFi credentials. The device will save these, restart, and connect to your local network.
4. **Access Dashboard:** The device's assigned IP address will appear on the TFT screen (under the WIFI INFO menu, accessed by tapping the WiFi icon). Enter this IP into any web browser to access the remote dashboard.

### BLE Trigger Setup
1. Turn on the device and wait for WiFi to connect. BLE initialization is deferred until WiFi stabilizes to preserve heap memory.
2. Go to the Bluetooth settings on your PC or camera control device.
3. Look for **ESP_Cam** and pair with it.
4. When the hardware trigger (Pin A4) is pulled LOW, the ESP32 will send an F12 keystroke over Bluetooth.

### Calibration
If you change optics or sensors, run the built-in calibration:
1. Tap **CALIBRATE** on the main screen.
2. Set your photo width (pixels) and demarcation distance (mm).
3. Follow the on-screen prompts to capture multiple points (3 to 20) at varying distances.
4. The device will calculate a linear regression, saving the slope, intercept, and R² to non-volatile memory.
# AutoFOV V11 Patched 3

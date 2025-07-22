# ESP32 I2S Microphone FFT Visualizer

This project reads audio data from an I2S microphone using an **ESP32** and performs a **Fast Fourier Transform (FFT)** on the data to visualize frequency magnitudes in real time on a small **OLED display (SSD1306, 128x32)**.

## 📋 Features
- Captures audio via **I2S** interface.
- Uses **arduinoFFT** library for frequency analysis.
- Displays real-time **FFT bar graph** on OLED.
- Prints FFT data over serial for debugging.

---

## 🛠 Hardware
| Component     | Purpose                 |
|---------------|--------------------------|
| **ESP32**     | Microcontroller           |
| **I2S Mic**   | Audio input (e.g., INMP441)|
| **SSD1306 OLED (128x32)** | Display frequency data |

---

## 💾 Software
### Libraries Used
- `arduinoFFT`
- `Adafruit_GFX`
- `Adafruit_SSD1306`
- `Wire`
- `driver/i2s.h` (ESP32 I2S peripheral)

---

## 📊 How it works
1. Audio data is collected via **I2S** at **44.1kHz, 16-bit**.
2. **512 samples** are collected per FFT frame.
3. A **Hamming window** is applied to minimize spectral leakage.
4. **FFT** is computed.
5. The magnitude spectrum is displayed as **128 vertical bars** on the OLED.



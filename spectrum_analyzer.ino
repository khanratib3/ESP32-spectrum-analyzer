
#include <driver/i2s.h>                  // ESP32 I2S peripheral driver
#include <arduinoFFT.h>                  // FFT library
#include <Wire.h>                        // I2C library
#include <Adafruit_GFX.h>                // Graphics library for OLED
#include <Adafruit_SSD1306.h>            // OLED driver library

// I2S pin definitions
#define WS 25                             // Word select (L/R clock)
#define SD 32                             // Serial data input
#define SCK 33                            // Serial clock

// OLED display parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Initialize OLED display

// I2S configuration
#define I2S_PORT I2S_NUM_0
#define bufferLen 512
int16_t sBuffer[bufferLen];              // Buffer to store I2S audio data

// FFT configuration
#include <arduinoFFT.h>
const uint16_t samples = 512;             // Number of samples (must be power of 2)
const float signalFrequency = 44100;      // Placeholder, not used in this example
const float samplingFrequency = 44100;    // Sampling frequency for I2S
const uint8_t amplitude = 100;            // Placeholder, not used in this example

float vReal[samples];                     // Real part of the FFT input/output
float vImag[samples];                     // Imaginary part of the FFT input/output (initialized to 0)

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency); // FFT object

// Function to install and configure I2S
void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),        // Master mode, RX only
    .sample_rate = 44100,                                     // Sample rate in Hz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,             // 16-bit audio
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,              // Left channel only
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,          // I2S MSB format
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,                                       // Number of DMA buffers
    .dma_buf_len = bufferLen,                                 // Length of each DMA buffer
    .use_apll = false,                                        // Don't use APLL
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);         // Install I2S driver
}

// Set the I2S pins
void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = SCK,              // Bit clock pin
    .ws_io_num = WS,                // Word select pin
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = SD               // Data input pin
  };

  i2s_set_pin(I2S_PORT, &pin_config); // Apply the pin configuration
}

// Function to draw FFT magnitude as bars on OLED
void drawFFT(float* vReal) {
  display.clearDisplay(); // Clear previous frame

  // Draw 128 bars (1-pixel wide, 128-pixel wide screen)
  for (int k = 0; k < samples/2; k++) {
    int barHeight = min(128, (int)(vReal[k] / 800)); // Scale magnitude for OLED height
    display.fillRect(k, SCREEN_HEIGHT - barHeight, 1, barHeight, SSD1306_WHITE); // Draw vertical bar
  }

  display.display(); // Push to screen
}

void setup() {
  Serial.begin(115200); // Initialize serial for debugging
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }

  Serial.println("Starting I2S Mic...");

  delay(1000); // Give some time before starting
  i2s_install(); // Configure I2S
  i2s_setpin();  // Set I2S pins
  i2s_start(I2S_PORT); // Start I2S peripheral
  delay(500);
}

void loop() {
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, sizeof(sBuffer), &bytesIn, portMAX_DELAY); // Read from I2S
  
  if (result != ESP_OK) {
    Serial.println("i2s_read() failed");
    return;
  }

  int16_t samples_read = bytesIn / sizeof(int16_t); // Number of samples read
  if (samples_read > 0) {
    long sum = 0;
    for (int16_t i = 0; i < samples_read; i++) {
      sum += sBuffer[i]; // Accumulate samples for mean
      vReal[i] = sBuffer[i]; // Populate FFT input (real part)
      vImag[i] = 0; // Imaginary part must be zero initially
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Apply window function
    FFT.compute(FFTDirection::Forward); // Perform FFT
    FFT.complexToMagnitude(); // Convert complex result to magnitude
    
    float x = FFT.majorPeak(); // (Optional) Find dominant frequency

    drawFFT(vReal); // Display FFT results on OLED

    float mean = (float)sum / samples_read; // Calculate DC bias (optional / unused here)

    Serial.println("FFT Results (Magnitude):");
    for (int i = 0; i < samples / 2; ++i) { // Only positive frequencies
      Serial.print("\t");
      Serial.println(vReal[i]); // Print magnitude of frequency bin
    }
  } else {
    Serial.println("No samples read"); // No data available
  }
}

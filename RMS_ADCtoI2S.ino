#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

AudioInputAnalog         adc1(A2);          // ADC object to record signal
AudioOutputI2S           i2s1;              // Define a I2S digital output object
AudioAnalyzeRMS          rms1;              // Define a RMS analysis object
AudioConnection          patchCord1(adc1, 0, i2s1, 0); // Connect ADC to left output path
AudioConnection          patchCord2(adc1, 0, i2s1, 1); // Connect ADC to right output path
AudioConnection          patchCord3(adc1, rms1);       // Connect ADC to RMS
AudioControlSGTL5000     sgtl5000_1;                   // Define Audio Adaptor Board control object

void setup() {
  Serial.begin(9600);

  AudioMemory(60); // Deliver memory

  sgtl5000_1.enable();    // enable Audio Adaptor Board
  // sgtl5000_1.volume(0.5); // set volumn
}

void loop() {
  // Check if there are RMS value available
  if (rms1.available()) {
    float rmsValue = rms1.read(); // Read RMS
    Serial.println(rmsValue);     
    delay(10);                   
  }
}



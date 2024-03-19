#include <Audio.h>

AudioInputAnalog         adc1(A2);
AudioOutputI2S           i2s1;
AudioRecordQueue         queue;    // Use a queue object to record data into the buffer
AudioConnection          patchCord1(adc1, 0, i2s1, 0);
AudioConnection          patchCord2(adc1, 0, i2s1, 1);
AudioConnection          patchCord3(adc1, queue);
AudioControlSGTL5000     sgtl5000_1;

void setup() {
  Serial.begin(9600);
  AudioMemory(60);

  sgtl5000_1.enable();
  // sgtl5000_1.volume(0.5);

  queue.begin();
}

void loop() {
  // Output original data
  if (queue.available() > 0) {
    int16_t *buffer = queue.readBuffer();  // read the buffer
    // for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    Serial.println(*buffer);
    delay(10);
      
    queue.freeBuffer();
    
  }
}

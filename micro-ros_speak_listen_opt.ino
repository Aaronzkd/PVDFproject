#include <micro_ros_arduino.h>
#include <vector>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>
// Message type lib
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>
// Audio lib
#include <Audio.h>
#include <SerialFlash.h>
#include <utility/imxrt_hw.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define LED_PIN 13

// Initialization
rcl_publisher_t publisher_sensors;
std_msgs__msg__Int16MultiArray msg;   // Define message type
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Definition of Receiver
AudioInputI2S            i2s;      // Audio shield communication
AudioRecordQueue         queue;    // Use a queue object to record data into the buffer
AudioConnection          patchCord1(i2s, queue);
AudioControlSGTL5000     sgtl5000_1;     //Audio shield
const int myInput = AUDIO_INPUT_LINEIN;   // Receiver input: line in

// Definition of Speaker
AudioSynthWaveform       waveform1;      //xy=188,240
AudioOutputI2S           i2s1;           //xy=565,241
AudioConnection          patchCord2(waveform1, 0, i2s1, 0);
AudioConnection          patchCord3(waveform1, 0, i2s1, 1);

/////////////////////////// Parameters Setting//////////////////////////////////
float TEE_SAMP_RATE = 44100;           // Hz, i2s sampling frequency
float MSG_SAMP_RATE = 400;             // Hz, how many messages are sent per second;  > TEE_SAMP_RATE/128 
const int WAVEFORM = WAVEFORM_SINE;    // WAVEFORM_SQUARE, WAVEFORM_SINE, WAVEFORM_PULSE, WAVEFORM_TRIANGLE, WAVEFORM_SAWTOOTH
float WAVE_Amp = 1;                    // [0, 1]
float WAVE_Freq = 10000;               // Hz, hearable: [13000, 16000] 
////////////////////////////////////////////////////////////////////////////////


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setI2SFreq(int freq) {
  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  double C = ((double)freq * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
       | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
       | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f 
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);  // casts last_call_time to void, see https://github.com/ros2/rclc/blob/0b90b3f725db666b892375cb6fe9f5b39f987c45/rclc/include/rclc/types.h
  if (timer != NULL)
  {    
    if (queue.available() > 0) {
        // Record buffer data
        int16_t *buffer = queue.readBuffer();  // read buffer from queue
        // Define parameters for msg.data
        msg.data.size = AUDIO_BLOCK_SAMPLES;
        msg.data.capacity = AUDIO_BLOCK_SAMPLES;

        if (msg.data.data != NULL) {
            // Transfer data to ROS message
            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                Serial.println(buffer[i]);
                msg.data.data[i] = buffer[i];
            }
            //  This publishes the message (msg)
            RCSOFTCHECK(rcl_publish(&publisher_sensors, &msg, NULL));
            free(msg.data.data);   // free message
            }
        queue.freeBuffer();
        }
  }
}

void setup()
{
  Serial.begin(9600);
  // Call this before anything else
  set_microros_transports();

  // Add your setup here
  AudioMemory(60);

  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.9);
  sgtl5000_1.lineOutLevel(16);    // 13-31, default 29

  setI2SFreq(TEE_SAMP_RATE);  // Sampling frequency: > 50kHz
  queue.begin();

  waveform1.begin(WAVE_Amp, WAVE_Freq, WAVEFORM);   // Set Waveforms: amplitude, frequency, waveform

  // Allocate dynamic memory for message. All values intialized to 0
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_basic_type_sequence_capacity = AUDIO_BLOCK_SAMPLES;

   bool success = micro_ros_utilities_create_message_memory(
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
       &msg,
       conf);

  if (!success)
  {
    error_loop();
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  allocator = rcl_get_default_allocator();

  // create init_options -- if no agent is running, RCL_RET_ERROR is returned. Try until agent connection established
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
  {
    delay(100);
  }

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create sensors publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_sensors,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "micro_ros_arduino_node_publisher"));

  // create timer -- timer_timeout is the minimum time that timer will run for 
  const unsigned int timer_timeout = RCL_US_TO_NS( (1 / MSG_SAMP_RATE) * 1000000 );  // 1000us=1000Hz -- 10000us=100Hz, 666us=1500Hz

  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      timer_timeout,
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

#include "main.h"





void setup() {
  // UART configuration

  //Set UART log level
	// esp_log_level_set(TAG, LOG_LEVEL);



  

  // Set up the PWM functionality
  // ledcSetup(MBB_PWM_1_CHANNEL, MBB_PWM_FREQ, MBB_PWM_RESOLUTION);
  // ledcSetup(MBB_PWM_2_CHANNEL, MBB_PWM_FREQ, MBB_PWM_RESOLUTION);

  // // Attach the PWM channels to the specified GPIO pins
  // ledcAttachPin(MBB_PWM_1_IO, MBB_PWM_1_CHANNEL);
  // ledcAttachPin(MBB_PWM_2_IO, MBB_PWM_2_CHANNEL);

  // Set the PWM duty cycle to the default value
  // ledcWrite(MBB_PWM_1_CHANNEL, MBB_PWM_VAL_SPEED_ZERO);
  // ledcWrite(MBB_PWM_2_CHANNEL, MBB_PWM_VAL_SPEED_ZERO);


}

void loop() {
  mbb_main();
}

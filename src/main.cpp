#include "main.h"
#include "mbb_mc.h"

void setup() {
  // UART configuration

  //Set UART log level
	// esp_log_level_set(TAG, LOG_LEVEL);



  

  // Config PWM
  // ledcSetup(MBB_MC_PWM_1_CHANNEL, MBB_MC_PWM_FREQ, MBB_MC_PWM_RESOLUTION);
  // ledcSetup(MBB_MC_PWM_2_CHANNEL, MBB_MC_PWM_FREQ, MBB_MC_PWM_RESOLUTION);

  // // Attach the channel to the GPIO to be controlled
  // ledcAttachPin(MBB_MC_PWM_1_IO, MBB_MC_PWM_1_CHANNEL);
  // ledcAttachPin(MBB_MC_PWM_2_IO, MBB_MC_PWM_2_CHANNEL);

  // // Set the PWM duty cycle to the default value
  // ledcWrite(MBB_MC_PWM_1_CHANNEL, MBB_MC_PWM_VAL_SPEED_ZERO);
  // ledcWrite(MBB_MC_PWM_2_CHANNEL, MBB_MC_PWM_VAL_SPEED_ZERO);

  // delay(2000);

  // // pwm 1 max positive
  // ledcWrite(MBB_MC_PWM_1_CHANNEL, MBB_MC_PWM_VAL_SPEED_MAX_POS);
  // // pwm 2 zero
  // ledcWrite(MBB_MC_PWM_2_CHANNEL, MBB_MC_PWM_VAL_SPEED_ZERO);
}

void loop() {
  mbb_main();

  // delay(1000);  
}

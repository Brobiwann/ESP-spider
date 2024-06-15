#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at 50 Hz updates

#define PI 3.1415926535897932384626433832795

// define servos locations pins on the PCA9685
uint8_t servo_channels[8] = {0, 1, 2, 3, 4, 5, 6, 7};


float period_rota = 0.6;
float period_lift = period_rota/2;


uint16_t initial_positions[8] = {280, 270, 280, 310, 280, 260, 280, 310};
float phase_offset[8] = {PI+PI/2, 0, PI/2, PI, PI+PI/2 , PI, PI/2, 0};
float period[8] = {period_lift, period_rota, period_lift, period_rota, period_lift, period_rota, period_lift, period_rota};
int amplitude[8] = {25, 30, 25, 30, 25, 30, 25, 30};

uint16_t previous_pwm_values[8] = {280, 380, 280, 300, 280, 280, 280, 280};

// function to calculate the pulse width of the servos depending on the sine wave
// we set the amplitude to the pulse width amplitude we want, and the period to the period in seconds that we want too
// the initial_pos is the initial position of the servos in pulsewidth

uint16_t calculate_servo_pulse(float period, float amplitude, float phase, int initial_pos) {

  float time_elapsed = millis() / 1000.0;

  // Getting the frequency using the period (the period is the time for a full movement of a servo)
  float frequency = (2 * PI) / period;
  
  //creating the sinusoidal function
  float angle = amplitude * sin(frequency * time_elapsed + phase) + initial_pos;

return angle;

}



void setup() {

  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Setting the oscillator to match the clock of the PCA9685
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

// Moving servos to initial positions
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(servo_channels[i], 0, initial_positions[i]);
  }
  delay(500); // Ensure servos have time to reach initial positions
}




void loop() {
    // Get the current time and add half a period in ms to offset the process
    unsigned long current_time = millis();

    for (int i = 0; i < 8; i++) {
        uint16_t current_pwm_value = calculate_servo_pulse(period[i], amplitude[i], phase_offset[i], initial_positions[i]);

        if (i==0 || i==2) {

          uint16_t pwm_rotation = calculate_servo_pulse(period[i+1], amplitude[i+1], phase_offset[i+1], initial_positions[i+1]);
          if (pwm_rotation<previous_pwm_values[i+1]){
            pwm.setPWM(servo_channels[i], 0, current_pwm_value);        
          }
        }

        
        else if (i==4 || i==6) {

          uint16_t pwm_rotation = calculate_servo_pulse(period[i+1], amplitude[i+1], phase_offset[i+1], initial_positions[i+1]);
          if (pwm_rotation>previous_pwm_values[i+1]){
            pwm.setPWM(servo_channels[i], 0, current_pwm_value);        
          }
        }

          
        else {pwm.setPWM(servo_channels[i], 0, current_pwm_value);}

        previous_pwm_values[i] = current_pwm_value;

        }
}
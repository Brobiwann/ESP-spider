//ESP-SPIDER
//The robot is a small 4 legged robot, two servos are used per leg
//The idea is inspired by the minikame by JavierIH https://github.com/JavierIH/miniKame
//
// This code enables the robot to walk in different directions using a PS4 controller. (Thanks to this amazing library : https://github.com/aed3/PS4-esp32)
// The robot's movement is controlled by eight servos (lift and rotation per leg) using a PCA9685 driver, 
// with each servo’s position updated through a sinusoidal function.
//
// The sinusoidal motion is calculated based on three parameters: amplitude, period, and phase offset, 
// which define the lift and rotation patterns for forward or turning movements. Each servo's pulse 
// width is adjusted to create smooth, cyclic motion, where the phase offset allows coordination 
// between servos for synchronized steps. 
//
// The eye of the robot is an Adafruit 8x8 LED Matrix w/I2C Backpack, it's also controlled by I2C
// The eye moves right and left when the robot turns, 
// And it's moving 1 pixel left and right in sync with the movement when the robot is going forward

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <PS4Controller.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at 50 Hz updates

#define PI 3.1415926535897932384626433832795


int eye_pos[2] = {3, 2}; // Initial eye position

uint8_t servo_channels[8] = {0, 1, 2, 3, 4, 5, 6, 7}; // define servos locations pins on the PCA9685

// positions of the servos on the robot :
//0 = back left lift
//1 = back left rotation
//2 = front left lift
//3 = front left rotation
//4 = front right lift
//5 = front right rotation
//6 = back right lift
//7 = back right rotation

uint16_t initial_positions[8] = {265, 260, 295, 310, 250, 250, 300, 300};

float period_rota = 0.6;
float period_lift = period_rota/2;


float phase_offset[8];
float period[8];
int amplitude[8];


uint16_t current_pwm_value = 0;
uint16_t previous_pwm_values[8];




// Function to calculate the next pulse width of each servo.
uint16_t calculate_servo_pulse(float period, float amplitude, float phase, int initial_pos) {

  float time_elapsed = millis() / 1000.0;

  // Getting the frequency using the period (the period is the time for a full movement of a servo)
  float frequency = (2 * PI) / period;
  
  //creating the sinusoidal function
  float pulse_width = amplitude * sin(frequency * time_elapsed + phase) + initial_pos;

return pulse_width;
}




//Function to set the parameters of the sinusoidal function for the robot to go forward
void Set_Parameters_Up() {

  float Up_phase_offset[8] = {PI+PI/2, 0, PI/2, PI, PI+PI/2 , PI, PI/2, 0};
  float Up_period[8] = {period_lift, period_rota, period_lift, period_rota, period_lift, period_rota, period_lift, period_rota};
  int Up_amplitude[8] = {27, 38, 27, 35, 27, 35, 27, 35};

  for (int i = 0; i < 8; i++) {
    phase_offset[i] = Up_phase_offset[i];
    period[i] = Up_period[i];
    amplitude[i] = Up_amplitude[i];
  }
}

//Function to set the parameters of the sinusoidal function for the robot to go right or left
//The parameters of the function are the same when the robot turn right or left
//In the main loop we just change the code so the legs stay in contact with the floor when the leg is turning clockwise or counter-clockwise
void Set_Parameters_Rotation() {

  float rotation_phase_offset[8] = {PI+PI/2, 0, PI/2, PI, PI+PI/2 , 0, PI/2, PI};
  float rotation_period[8] = {period_lift, period_rota, period_lift, period_rota, period_lift, period_rota, period_lift, period_rota};
  int rotation_amplitude[8] = {30, 25, 30, 25, 30, 25, 30, 25};

  for (int i = 0; i < 8; i++) {
    phase_offset[i] = rotation_phase_offset[i];
    period[i] = rotation_period[i];
    amplitude[i] = rotation_amplitude[i];
  }
}




void MoveEyeRight() {
  matrix.clear();
  matrix.drawRect(eye_pos[0] - 2, eye_pos[1], 2, 4, LED_ON);
  matrix.writeDisplay();
}

void MoveEyeLeft() {
  matrix.clear();
  matrix.drawRect(eye_pos[0] + 2, eye_pos[1], 2, 4, LED_ON);
  matrix.writeDisplay();
}

void MoveEyeCenter() {
  matrix.clear();
  matrix.drawRect(eye_pos[0],eye_pos[1], 2, 4, LED_ON);
  matrix.writeDisplay();
}

//Function to make the eye move right and left when the robot is walking
//The movement of the eye is linked to the movement of the back left rotation servo
void MoveEyeWalking(int i) {
  if (i==1) {
    if (current_pwm_value<previous_pwm_values[1]) {                            //Check if the leg is going CCW
      matrix.clear();                                                          //if so, clear matrix
      matrix.drawRect(eye_pos[0]+1,eye_pos[1], 2, 4, LED_ON);                  //move the eye to the left
      matrix.writeDisplay();                                                   //update led matrix display
    }            

    else if (current_pwm_value>previous_pwm_values[1]) {                       //Check if the leg is going CW
      matrix.clear();                                                          //if so, clear matrix
      matrix.drawRect(eye_pos[0]-1,eye_pos[1], 2, 4, LED_ON);                  //move the eye to the right
      matrix.writeDisplay();                                                   //update led matrix display
    }
  }
}





void setup() {

  Serial.begin(115200);
  matrix.begin(0x70);
  PS4.begin("e4:65:e8:75:7b:44"); //MAC address of the PS4 controller
  Serial.println("Ready.");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Setting the oscillator to match the clock of the PCA9685
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

// Moving servos to initial positions
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(servo_channels[i], 0, initial_positions[i]);
    delay(20);
  }
  delay(500); // Ensure servos have time to reach initial positions

}




void loop() {

  unsigned long currentMillis = millis();  // Track current time

  if (PS4.isConnected()) {

    if (PS4.Up()) {

      Serial.println("Up");
     
      Set_Parameters_Up(); // Set the offsets, amplitude, period for the robot to go forward

      for (int i = 0; i < 8; i++) {

        current_pwm_value = calculate_servo_pulse(period[i], amplitude[i], phase_offset[i], initial_positions[i]);

        MoveEyeWalking(i);

        //When going forward, for the lift servos 0 and 2,
        //we want those legs to stay in contact with the floor when the legs are turning CCW
        //So we get the the value of the rotation servo on the same leg
        //And we only update the servo if the leg is going CW
        if (i==0 || i==2) {
          uint16_t pwm_rotation = calculate_servo_pulse(period[i+1], amplitude[i+1], phase_offset[i+1], initial_positions[i+1]);
          if (pwm_rotation<previous_pwm_values[i+1]){
            pwm.setPWM(servo_channels[i], 0, current_pwm_value);      
          }
        }

        //When going forward, for the lift servos 4 and 6,
        //we want those legs to stay in contact with the floor when the legs are turning CW
        //So we get the the value of the rotation servo on the same leg
        //And we only update the servo if the leg is going CCW
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



    if (PS4.Right()) {

      Serial.println("Right");

      MoveEyeRight();

      Set_Parameters_Rotation(); // Set the offsets, amplitude, period for the robot to go Right

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
          if (pwm_rotation<previous_pwm_values[i+1]){
            pwm.setPWM(servo_channels[i], 0, current_pwm_value);        
          }
        }
    
        else {pwm.setPWM(servo_channels[i], 0, current_pwm_value);}

        previous_pwm_values[i] = current_pwm_value;

      }
    }



    if (PS4.Left()) {

      Serial.println("Left");

      MoveEyeLeft();

      Set_Parameters_Rotation(); // Set the offsets, amplitude, period for the robot to go left

     for (int i = 0; i < 8; i++) {
        uint16_t current_pwm_value = calculate_servo_pulse(period[i], amplitude[i], phase_offset[i], initial_positions[i]);

        if (i==0 || i==2) {

          uint16_t pwm_rotation = calculate_servo_pulse(period[i+1], amplitude[i+1], phase_offset[i+1], initial_positions[i+1]);
          if (pwm_rotation>previous_pwm_values[i+1]){
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

    //if no button is press, move the eye in the center
    //and mlove the servos to initial positions
    if(!PS4.data.button.up && !PS4.data.button.right && !PS4.data.button.left) {

      MoveEyeCenter();

     for (int i = 0; i < 8; i++) {
       pwm.setPWM(servo_channels[i], 0, initial_positions[i]);
     }

     delay(20);

    }
  }
}
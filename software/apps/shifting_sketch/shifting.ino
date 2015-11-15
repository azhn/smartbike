#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include "pwm.h"

//the pins
int LED = 13;
int WHEEL_TRIGGER = 2;

//some constants
unsigned long MM_PER_INT = 1100;// 977; 977 is true, trying different value to get better pedal speed

uint16_t test = 325;

int REAR_PWM = 0;
int FRONT_PWM = 1;
//uint16_t REAR_GEAR_UP[7] = {310, 340, 352, 365, 372, 340, 373};
uint16_t REAR_GEAR_UP[7] = {310, 340, 340, 352, 365, 372, 372};
uint16_t REAR_GEAR_DOWN[7] = {304, 340, 315, 327, 340, 372, 372};
//uint16_t REAR_GEAR_DOWN[7] = {304, 315, 327, 343, 372, 342, 372};
uint16_t FRONT_GEAR_UP[7] = {400, 400, test, test, test, test, test};
uint16_t FRONT_GEAR_DOWN[7] = {400, 440, test, test, test, test, test};
bool UP = true;
bool DOWN = false;

//our state
struct State{
  //store time when hall effect triggers interrupt
  unsigned long last_milli = 0;
  unsigned long curr_milli = 0;

  //some stuff for the shifting logic
  bool dir = UP;

  //to begin with try seven states.
  int target_gear_state = 0;
  int curr_gear_state = 0;
};

volatile struct State state;

//wheel interupt flag
bool wheel_int;

//our pwm object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

///////////////////////////////////////////////////////////////////////

void update_gears(int gear)
{
  if(gear > state.curr_gear_state)
  {
    state.dir = UP;
  }
  else if(gear < state.curr_gear_state)
  {
    state.dir = DOWN;
  }
  
  if(state.dir == UP)
  {
    pwm.setPWM(REAR_PWM, 0, REAR_GEAR_UP[gear]);
    pwm.setPWM(FRONT_PWM, 0, FRONT_GEAR_UP[gear]);
  }
  else
  {
    pwm.setPWM(REAR_PWM, 0, REAR_GEAR_DOWN[gear]);
    pwm.setPWM(FRONT_PWM, 0, FRONT_GEAR_DOWN[gear]);
  }
  state.curr_gear_state = gear;
}


///////////////////////////////////////////////////////////////////////

void setup() {
  //wheel interrrupt attatched to pin 2
  attachInterrupt(digitalPinToInterrupt(WHEEL_TRIGGER), interrupt, RISING);

  //LED and serial for debugging purposes
  pinMode(LED, OUTPUT);
  Serial.begin(9600);

  //wheel interrupt initially false
  wheel_int = false;

  //setup pwm driver
  pwm.begin();
  pwm.setPWMFreq(52); //20ms period
  pwm.setPWM(REAR_PWM, 0, REAR_GEAR_UP[0]);
  pwm.setPWM(FRONT_PWM, 0, FRONT_GEAR_UP[0]);
}

void loop() {

  //handle wheel interrupt
  if(wheel_int){
    //update the time interrupt was recieved.
    //this is not exactly accurate, but millis() can not be run from in an interrupt
    //also, accurate to several milliseconds is more than accurate enough for our purposes
    state.last_milli = state.curr_milli;
    state.curr_milli = millis();

    //update state
    //this will give us a state that is 1 - 7 for our gear states.

    //get speed m/s
    int mm_per_ms = MM_PER_INT / ( state.curr_milli - state.last_milli );
    state.target_gear_state = mm_per_ms >> 1;
    if(state.target_gear_state > 6) state.target_gear_state = 6;

    //update PWM
    update_gears(state.target_gear_state);
    
    //print delta_time for debuggin purposes
    Serial.println(state.target_gear_state);

    //clear wheel interrupt "flag"
    wheel_int = false;

  }
  
}

void interrupt(){
  wheel_int = true;
}

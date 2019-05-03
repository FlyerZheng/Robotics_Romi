
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Library Includes.                                                             *
 * Be sure to check each of these to see what variables/functions are made        *
 * global and accessible.                                                        *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "pid.h"
#include "interrupts.h"
#include "kinematics.h"
#include "line_sensors.h"
#include "irproximity.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Definitions.  Other definitions exist in the .h files above.                  *
 * Also ensure you check pins.h for pin/device definitions.                      *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 9600



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class Instances.                                                              *
 * This list is complete for all devices supported in this code.                 *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading
Kinematics    Pose_map;


LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

SharpIR       MidDistanceSensor(SHARP_IR_PIN); //Distance sensor
SharpIR       LeftDistanceSensor(Left_SHARP_IR_PIN); 
SharpIR       RightDistanceSensor(Right_SHARP_IR_PIN); 

Imu           Imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 3.5, 20.9, 0.04 );
PID           RightSpeedControl( 3.5, 20.9, 0.04 );
PID           HeadingControl( 1.5, 0, 0.001 );

Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Global variables.                                                             *
 * These global variables are not mandatory, but are used for the example loop() *
 * routine below.                                                                *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;
float left_speed_demand = 0;
float right_speed_demand = 0;

bool goForward;
bool step1 = true;
float set_Speed = 5;
float Y_offset;
bool obstacle;


float obstacle_x;
float obstacle_y;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This setup() routine initialises all class instances above and peripherals.   *
 * It is recommended:                                                            *
 * - You keep this sequence of setup calls if you are to use all the devices.    *
 * - Comment out those you will not use.                                         *
 * - Insert new setup code after the below sequence.                             *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void setup()
{

  // These two function set up the pin
  // change interrupts for the encoders.
  setupLeftEncoder();
  setupRightEncoder();
  startTimer();

  //Set speed control maximum outputs to match motor
  LeftSpeedControl.setMax(100);
  RightSpeedControl.setMax(100);

  // For this example, we'll calibrate only the 
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate();

  //Setup RFID card
  setupRFID();

  // These functions calibrate the IMU and Magnetometer
  // The magnetometer calibration routine require you to move
  // your robot around  in space.  
  // The IMU calibration requires the Romi does not move.
  // See related lab sheets for more information.
  /*
  Wire.begin();
  Mag.init();
  Mag.calibrate();
  Imu.init();
  Imu.calibrate();
  */

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));

  
  // Initialise Serial communication
  Serial.begin( BAUD_RATE );
  delay(1000);
  Serial.println("Board Reset");

  // Romi will wait for you to press a button and then print
  // the current map.
  //
  // !!! A second button press will erase the map !!!
  ButtonB.waitForButton();  

  Map.printMap();

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();  

  Serial.println("Map Erased - Mapping Started");
  Map.resetMap();

  // Your extra setup code is best placed here:
  float right_cover_speed = 100;
  float left_cover_speed = 0;
  // but not after the following:

  // Because code flow has been blocked, we need to reset the
  // last_time variable of the PIDs, otherwise we update the
  // PID with a large time elapsed since the class was 
  // initialised, which will cause a big intergral term.
  // If you don't do this, you'll see the Romi accelerate away
  // very fast!
  LeftSpeedControl.reset();
  RightSpeedControl.reset();
  left_speed_demand = 3;
  right_speed_demand = 3;


  goForward = true;
  
  obstacle = false;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This loop() demonstrates all devices being used in a basic sequence.          
 * The Romi should:                                                                              
 * - move forwards with random turns 
 * - log lines, RFID and obstacles to the map.
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int state = 0;
//#define state_covering 0
//#define state_check_obstacle 0
//#define state_obstacle_avoiding 2


void loop() {
  Pose.update();
  Pose_map.setPose(Pose.getX(), Pose.getY(), Pose.getThetaDegrees());
//  Pose.printPose();
  doMapping();
  float MidDistance = MidDistanceSensor.getDistanceInMM();
  if( MidDistance < 15 ){
    obstacle = true;
    }
  
  if (obstacle){
    switch(state){
      case 0: check_obstacle();break;
      case 1: run_obstacle();break;
    }
  }else{
    cover(); 
  }
}


void cover(){
  if(goForward){    
        if(Pose.getX() < 1661 & step1){ //1800 - 139 - 50 = 1611
          Serial.println(Pose.getX());      
          goStraight(set_Speed);
          
        } else {
          step1 = false;
         
          if(abs(Pose.getThetaDegrees()) < 83){
            turnRight(set_Speed);
            Y_offset = Pose.getY();
          } else {       
            if((abs(Pose.getY() - Y_offset)) < 140){
              goStraight(set_Speed);      
            } else {
    
              if(abs(Pose.getThetaDegrees()) < 168){
                turnRight(set_Speed);
              } else {
                goForward = false;
                step1 = true;
                Pose.resetPose();
              }       
            }
          }   
        }
   } 

   if(!goForward){    
    if(Pose.getX() < 1661 & step1){ //1800 - 139 - 50 = 1611            
      goStraight(set_Speed);
    } else {
      step1 = false;
      Pose.update();      
      if(abs(Pose.getThetaDegrees()) < 83){
        turnLeft(set_Speed);
        Y_offset = (Pose.getY());
      } else {        
        if((abs(Pose.getY() - Y_offset)) < 140){
          goStraight(set_Speed);      
        } else {          
          Pose.update();
          Serial.println(Pose.getThetaDegrees());
          if(abs(Pose.getThetaDegrees()) < 172.5){
            turnLeft(set_Speed);
          } else {
            goForward = true;
            step1 = true;
            Pose.resetPose();
          }       
        }
      }   
    }
   } 
  
  
  }



void turnLeft(float set_Speed) {
        left_speed_demand = -set_Speed;
        right_speed_demand = set_Speed;  
}

void turnRight(float set_Speed) {
        left_speed_demand = set_Speed;
        right_speed_demand = -set_Speed;  
}

void goStraight(float set_Speed) {
        left_speed_demand = set_Speed;
        right_speed_demand = set_Speed;  
}




void check_obstacle(){

  float MidDistance = MidDistanceSensor.getDistanceInMM();

  if( MidDistance < 15 ){
    left_speed_demand = 5;
    right_speed_demand = -5;
    delay(1000);
    obstacle_y = Pose.getY();
    obstacle_x = Pose.getX();
    state = 1;
    }
    
}






void run_obstacle(){
  int speed = 5;
  float LeftDistance = LeftDistanceSensor.getDistanceInMM();
  if( LeftDistance <  12 ){
    left_speed_demand = speed;
    right_speed_demand = -speed;
    }
    
  if( LeftDistance >17 ){
    left_speed_demand = -speed;
    right_speed_demand = speed;
      }
      
  if (LeftDistance > 12 && LeftDistance <17 ){
    left_speed_demand = speed;
    right_speed_demand = speed;
  }

   if (abs(Pose.getY() - obstacle_y) < 10 && abs(Pose.getX() - obstacle_x)>15){
    if (abs(Pose.getThetaDegrees())<5){
      left_speed_demand = 0;
      right_speed_demand = 0;
      obstacle = false;
      state = 0;
      }else{
        left_speed_demand = 4;
        right_speed_demand = -4;
        }
    }
}



void doMapping() {
  
  float distance = LeftDistanceSensor.getDistanceInMM();
  if( distance > 12 && distance <17  ) {
    distance += 80;
    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose_map.getX() + ( distance * cos( Pose_map.getThetaRadians() ) );
    float projected_y = Pose_map.getY() + ( distance * sin( Pose_map.getThetaRadians() ) );
    Map.updateMapFeature( (byte)'O', projected_x, projected_y );
    
    
  } 

  if( LineCentre.readRaw() > 580 ) {
      Map.updateMapFeature( (byte)'L', Pose.getY(), Pose.getX() );
  } 
}

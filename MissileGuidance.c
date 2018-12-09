//=================================
// Real time system
//=================================
// Missile Guidance
// Name: Xiaolou Huang (Lor)
// Summer 2018
//=================================
// 1. Get the elapse time t_setup_drone.
// 2. Do the calculations for the direction (angle) and speed of the drone.
//    The angle of the missile (the speed of missile is given, but you can change it).
//    The time for them to collided.
//    And so on. for more infomation, check out the variable declarations.
// 3. Display to OLED:
//      Dynamic output:
//        Drone's coordinates
//        Missile's coordinates.
//        Time they collide
//      static output:
//        Speed of the drone
//        Angle of the missile
//==================================
// User's mannual:
//   For Max32 micro-controller
//   SW1: start the time at P1
//   SW2: end the time at P2
//==================================

#include <IOShieldOled.h>

// set pin numbers:
const int BTN1 = 4;    
const int BTN2 = 78;    
const int BTN3 = 80;    
const int BTN4 = 81;

const int SW1 = 2;
const int SW2 = 7;
const int SW3 = 8;
const int SW4 = 79;   

const int ledPin =  13;     
const int LD1 =  70;      
const int LD2 =  71;    
const int LD3 =  72;
const int LD4 =  73;
const int LD5 =  74;
const int LD6 =  75;
const int LD7 =  76;
const int LD8 =  77;	 // System Operational LED

// state variables:
int BTN1_state = 0;       
int BTN2_state = 0;
int BTN3_state = 0;
int BTN4_state = 0;
int SW1_state = 0; 
int SW2_state = 0; 
int SW3_state = 0; 
int SW4_state = 0; 

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
// Note: for 10-bit A/D, the range of sensorValue = 0 to 1023

// variables declaration
// for this application, for distance in miles
int y0_setup_drone = 6;  // first detect point (5, y0_setup_drone)
int y1_setup_drone = 8;  // second detect point (7, y1_setup_drone)
float t_setup_drone = 0.0;  // set up time for drone to pass through points (5, y0) and (7, y1)
float t_intercept = 0.0;
float x_drone = 0.0;
float y_drone = 0.0;
float x_missile = 0.0;
float y_missile = 0.0;
float angle_drone = 0.0;
float speed_drone = 0.0;
float angle_missile = 0.0;
float speed_missile = 0.4267;  // given as twice fast as speed of sound (speed of sound = 768mph = 0.2133 mile/s)
// also essential variables
float setup_distance_drone = 0.0;
float speed_xdirection_drone = 0.0;
float distance_xdirection_missile = 0.0;
float distance_xdirection_drone = 0.0;
float distance_ydirection_missile = 0.0;
float distance_ydirection_drone = 0.0;
float distance_missile = 0.0;
float distance_drone = 0.0;
// dynamic incrementing variables
float x_drone_increment = 0.0;
float y_drone_increment = 0.0;
float x_missile_increment = 0.0;
float y_missile_increment = 0.0;
float t_intercept_increment = 0.0;


void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);  
  pinMode(LD1, OUTPUT);  
  pinMode(LD2, OUTPUT);    
  pinMode(LD3, OUTPUT);  
  pinMode(LD4, OUTPUT);     
  pinMode(LD5, OUTPUT);  
  pinMode(LD6, OUTPUT);    
  pinMode(LD7, OUTPUT);  
  pinMode(LD8, OUTPUT);     

  // initialize the pushbutton pin as an input:
  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT);
  pinMode(BTN4, INPUT);

  // initialize switches as inputs:
  pinMode(SW1, INPUT);  
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(SW4, INPUT); 

  // Turn OFF all LEDs
  digitalWrite(LD1, LOW); 
  digitalWrite(LD2, LOW); 
  digitalWrite(LD3, LOW); 
  digitalWrite(LD4, LOW);  

  digitalWrite(LD5, LOW); 
  digitalWrite(LD6, LOW); 
  digitalWrite(LD7, LOW); 
  digitalWrite(LD8, LOW); 

  // Initialize OLED:
  IOShieldOled.begin();
  IOShieldOled.displayOn();

}  // end setup()


void loop() {

  // System Operation LED ON:
  digitalWrite(LD8, HIGH);
  delay(10);           // wait for 250 ms 

  // read switches and buttons
  BTN1_state = digitalRead(BTN1);
  BTN2_state = digitalRead(BTN2);
  BTN3_state = digitalRead(BTN3);
  BTN4_state = digitalRead(BTN4);

  SW1_state = digitalRead(SW1);
  SW2_state = digitalRead(SW2);
  SW3_state = digitalRead(SW3);
  SW4_state = digitalRead(SW4);

  IOShieldOled.setCursor(0,0);
  char setup_time_charArr[15]; 
  snprintf(setup_time_charArr, 15, "t_elapse:%.2f", t_setup_drone); 
  //Display it @OLED:
  IOShieldOled.putString(setup_time_charArr);

  // cursor 0,1
  IOShieldOled.setCursor(0,1);
  char angle_missile_charArr[20];
  snprintf(angle_missile_charArr, 20, "mi_angle:%.2f", angle_missile);
  //Display it @ OLED:
  IOShieldOled.putString(angle_missile_charArr);

  // cursor 0,2
  IOShieldOled.setCursor(0,2);
  char t_intercept_and_speed_drone_charArr[20];
  snprintf(t_intercept_and_speed_drone_charArr, 20, "Vd:%.2f|t:%.2f ", speed_drone, t_intercept_increment);
  //Display it @OLED:
  IOShieldOled.putString(t_intercept_and_speed_drone_charArr);

  // cursor 0,3
  // dynamic increment setting for missile
  IOShieldOled.setCursor(0,3);
  char missileDrone_coordinates_charArr[20];
  snprintf(missileDrone_coordinates_charArr, 20, "mi_co:%.2f,%.2f", x_missile_increment, y_missile_increment);    
  //Display it @OLED:
  IOShieldOled.putString(missileDrone_coordinates_charArr);

  //--------------------------------------------------------------
  // 1. Get the elapse time, t_setup_drone.

  if (SW1_state == HIGH && SW2_state == LOW) {
    t_setup_drone += 0.02;

    IOShieldOled.setCursor(0,0);
    char setup_time_charArr[15]; 
    snprintf(setup_time_charArr, 15, "t_elapse:%.2f", t_setup_drone); 
    //Display it @OLED:
    IOShieldOled.putString(setup_time_charArr);
  }

  if (SW2_state == HIGH) {

    IOShieldOled.setCursor(0,0);
    char setup_time_charArr[15]; 
    snprintf(setup_time_charArr, 15, "t_elapse:%.2f", t_setup_drone); 
    //Display it @OLED:
    IOShieldOled.putString(setup_time_charArr);

    //--------------------------------------------------------------
    // 2. Do the calculations for the direction (angle) and speed of the drone.
    //    The angle of the missile (the speed of missile is given, but you can change it).
    //    The time for them to collided
    //----------------------------
    // calculation for drone 
    // find the speed of the drone
    setup_distance_drone = sqrt(pow((y1_setup_drone - y0_setup_drone), 2) + pow((7 - 5), 2));
    speed_drone = setup_distance_drone / t_setup_drone;

    angle_drone = atan((y1_setup_drone - y0_setup_drone) / (7 - 5)) * 180 / PI;  // drone angle in degree
    speed_xdirection_drone = speed_drone * cos(angle_drone * PI / 180);  // drone speed in x direction
    //----------------------------
    // speed_xdirection_missile = speed_missile * cos(angle_missile)
    // 1. xdistance_missile = xdistance_drone
    //    7 + speed_missile * cos(angle_missile) * time = 7 + speed_xdirection_drone * time
    // 2. after cancellation, solve for angle_missile 
    // 3. angle_missile = arccos(speed_xdirection_drone) / speed_missile
    angle_missile = acos(speed_xdirection_drone / speed_missile) * 180 / PI;  // in degree

    // equation of drone:
    // distance_ydirection_drone - y0_setup_drone = (y1_setup_drone - y0_setup_drone) / (7 - 5) * (distance_xdirection_drone - 5)
    // equation of missile:
    // distance_ydirection_missile = tan(angle_missile) * (distance_xdirection_missile - 7)

    // distance_ydirection_drone = distance_ydirection_missile. (distance_xdirection_drone = distance_xdirection_missile)
    // after munipulation on both sides of equation, solve for distance_xdirection_missile
    distance_xdirection_missile = (((-7) * tan(angle_missile * PI / 180)) + ((y1_setup_drone - y0_setup_drone) * 7 / 2) - y1_setup_drone) / ((y1_setup_drone - y0_setup_drone) / 2 - tan(angle_missile * PI / 180)) - 7;
    distance_xdirection_drone = distance_xdirection_missile;
    distance_ydirection_missile = distance_xdirection_missile * tan(angle_missile * PI / 180);
    distance_ydirection_drone = distance_xdirection_drone * tan(angle_drone * PI / 180);

    // solve for time intercept
    distance_missile = sqrt(pow(distance_xdirection_missile, 2) + pow(distance_ydirection_missile, 2));
    distance_drone = sqrt(pow(distance_xdirection_drone, 2) + pow(distance_ydirection_drone, 2));
    t_intercept = distance_missile / speed_missile; 

    //--------------------------------------------------------------
    // 3. Display to OLED:
    //      Dynamic output:
    //        Drone's coordinates
    //        Missile's coordinates.
    //        Time they collide
    //      static output:
    //        Speed of the drone
    //        Angle of the missile 

      // check if the drone moves fast than missile 
    if (speed_drone >= speed_missile) {
      IOShieldOled.setCursor(0,1);
      //Display it @OLED:
      IOShieldOled.putString("Drone moves");
      IOShieldOled.setCursor(0,2);
      //Display it @OLED:
      IOShieldOled.putString("too FAST");
    }
    else {

      float increment = 0.02;  // set incrementing value

      // cursor 0,1
      IOShieldOled.setCursor(0,1);
      char angle_missile_charArr[20];
      snprintf(angle_missile_charArr, 20, "mi_angle:%.2f", angle_missile);
      //Display it @ OLED:
      IOShieldOled.putString(angle_missile_charArr);

      // cursor 0,2
      if (t_intercept_increment <= t_intercept) {
        t_intercept_increment += increment; 
      }
      IOShieldOled.setCursor(0,2);
      char t_intercept_and_speed_drone_charArr[20];
      snprintf(t_intercept_and_speed_drone_charArr, 20, "Vd:%.2f|t:%.2f ", speed_drone, t_intercept_increment);
      //Display it @OLED:
      IOShieldOled.putString(t_intercept_and_speed_drone_charArr);

      // cursor 0,3
      // dynamic increment setting for missile
      if (x_missile_increment <= distance_xdirection_missile) {
        x_missile_increment += (distance_xdirection_missile / (t_intercept / increment));
      }
      if (y_missile_increment <= distance_ydirection_missile) {
        y_missile_increment += (distance_ydirection_missile / (t_intercept / increment)); 
      }
      IOShieldOled.setCursor(0,3);
      char missileDrone_coordinates_charArr[20];
      snprintf(missileDrone_coordinates_charArr, 20, "mi_co:%.2f,%.2f", x_missile_increment, y_missile_increment);    
      //Display it @OLED:
      IOShieldOled.putString(missileDrone_coordinates_charArr);
    }

  } // end SW2_state == HIGH

  IOShieldOled.updateDisplay(); 
  //clear the display and reset the cursor to zero
  IOShieldOled.clearBuffer();

  //----------------------------------------
  // System Operation LED Indicator:
  digitalWrite(LD8, LOW);  
  delay(10);           // wait for 250 ms 

}
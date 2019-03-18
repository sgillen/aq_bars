//#if ARDUINO >= 100
#include "Arduino.h"
//#else
//#include "WProgram.h"
//#end if

#include <Servo.h>
#include <math.h>

//uncomment for digital servos in the Shoulder and Elbow
//that use a range of 900ms to 2100ms
//#define DIGITAL_RANGE

//Arm Servo pins
#define Base_pin 2
#define Shoulder_pin 4
#define Elbow_pin 6
#define Wrist_pin 8
#define Gripper_pin 10
#define WristR_pin 12

//Onboard Speaker
#define Speaker_pin 5

#define BUFFER_SIZE 32 //may need to change

#define X_MAX 11
#define X_MIN 5.5
#define Y_MAX 6
#define Y_MIN -2.5
#define Z_MAX 90
#define Z_MIN 0

char buffer[BUFFER_SIZE]; //this is the buffer where we store incoming text from the computer
uint16_t serialBufferPos;

const float A = 5.75;
const float B = 7.375;

//Radians to Degrees constant
const float rtod = 57.295779;

//Arm Speed Variables
float Speed = 1.0;
int sps = 3;

//Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo Gripper;
Servo WristR;

//Arm Current Pos
float X = 0;
float Y = 0;
float Z = 0;
int G = 90;
float WA = 0;
int WR = 90;

//Arm temp pos
float x = 5.5;   //5.5 - 11
float y = 4.0;   // -2.5 - 6
float z = 45;   // 0 - 90
int g = 90;
int wr = 90;
float wa = 0;

//boolean mode = true;


// The arduino does NOT have a floating point unit, we may need to do these calculations on the laptop and send over joint position
int Arm(float x, float y, float z, int g, float wa, int wr) //Here's all the Inverse Kinematics to control the arm
{
  float M = sqrt((y*y)+(x*x));

  Serial.print("M = "); Serial.println(M, DEC);
  
  if(M <= 0)
    return 1;
  float A1 = atan(y/x);
  
  Serial.print("A1 = "); Serial.println(A1, DEC);

  if(x <= 0)
    return 1;
    
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;

  Serial.print("elb = "); Serial.print(Elbow, DEC); Serial.print("\t shoul = "); Serial.print(Shoulder, DEC); Serial.print("\t z = "); Serial.print(z, DEC);

  
  if((int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  float Wris = abs(wa - Elbow - Shoulder) - 90;
#ifdef DIGITAL_RANGE
  Elb.writeMicroseconds(map(180 - Elbow, 0, 180, 900, 2100  ));
  Shldr.writeMicroseconds(map(Shoulder, 0, 180, 900, 2100));
#else
  Elb.write(180 - Elbow);
  Shldr.write(Shoulder);
#endif
  Wrist.write(180 - Wris);
  Base.write(z);
  WristR.write(wr);
#ifndef FSRG
  Gripper.write(g);
#endif
  Y = y;
  X = x;
  Z = z;
  WA = wa;
#ifndef FSRG
  G = g;
#endif


  return 0; 
}

void setup()
{
  Serial.begin(9600);
  Base.attach(Base_pin);
  Shldr.attach(Shoulder_pin);
  Elb.attach(Elbow_pin);
  Wrist.attach(Wrist_pin);
  Gripper.attach(Gripper_pin);
  WristR.attach(WristR_pin);
  Arm(x, y, z, g, wa, wr);
}

void loop(){
    if(Serial.available() > 0){
  
         // Read next byte from serial into buffer
        buffer[serialBufferPos] = Serial.read();
    
        // Serial.print("buffer is: ");
        // Serial.println(buffer);

        // Check if we've reached exclamation
        if (buffer[serialBufferPos] == '!') {

            char* prot = strtok(buffer, ","); //prot tells us what cmd was sent
            if ( !prot[0] ) { //check if something about the packet is malformed enough that strok fails
                Serial.println("B1");
            }
            
            // Handle specific commands
            else if ( prot[0] == 'm' ) {
  
               // converts our values from a string x,y,z! to a float
               x = atof(strtok(NULL, ","));
               y = atof(strtok(NULL, ","));
               z = atof(strtok(NULL, "!"));
              
               // truncate our values if needed
               x =  x < X_MIN ? X_MIN : x; 
               x =  x > X_MAX ? X_MAX : x;
              
               y =  y < Y_MIN ? Y_MIN : y;
               y =  y > Y_MAX ? Y_MAX : y;
              
               z =  z < Z_MIN ? Z_MIN : z;
               z =  z > Z_MAX ? Z_MAX : z;
               
               // Display position
               Serial.print("x = "); Serial.print(x, DEC); Serial.print("\t y = "); Serial.print(y, DEC); Serial.print("\t z = "); Serial.print(z, DEC); Serial.print("\t g = "); Serial.print(g, DEC); Serial.print("\t wa = "); Serial.print(wa, DEC); Serial.print("\t wr = "); Serial.println(wr, DEC);

                  
               // Move arm
               Arm(x, y, z, g, wa, wr);
            
            }else if (prot[0] == 's'){
                  //TODO shutoff commands
                  
            }else { // Bad command
                Serial.println("B2");
                Serial.println(prot[0]);
            }
    
        // Reset buffer position
        serialBufferPos = 0;
        buffer[0] = 0;

       }

    else {
      Serial.print("Buffer pos ");
      Serial.println(serialBufferPos);
      serialBufferPos++;
    }
    
  }
}


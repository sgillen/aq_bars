//////////////////////////////////////////////////////////
// ROSfalcon Driver. Publishes and subscribes to falconMsgs for Novint Falcon.
//
// Using LibniFalcon 
// Steven Martin
// Based on barrow_mechanics example by Alistair Barrow
// modified by Sean Gillen


#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <aqbar/falconPos.h>
#include <aqbar/falconForces.h>


#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

#include <std_msgs/Byte.h>
#include "serial/serial.h"

#include <unistd.h>


using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

#define X_MAX 11
#define X_MIN 5.5
#define Y_MAX 6
#define Y_MIN -2.5
#define Z_MAX 90
#define Z_MIN  0

#define FX_MAX .05
#define FX_MIN -.05
#define FY_MAX .055
#define FY_MIN -.055
#define FZ_MAX .17
#define FZ_MIN .07

#define COUNT_MAX 100;// how many IO cycles to we count a sensed wall as valid

#define WALL_K 1; // how stiff our virtual walls appear

FalconDevice g_falconDevice;


// If I was being good I would make all of this into a class and avoid globals
// completely but I don't really feel like rejigging the falcon driver code and
// for now the codebase is small enough an isolated enough that it really does
// not matter

std::array<double, 3> g_pos;

double g_right_wall_pos;
double g_left_wall_pos;
double g_top_wall_pos;
double g_bottom_wall_pos;

int g_right_wall_count = 0;
int g_left_wall_count = 0;
int g_top_wall_count = 0;
int g_bottom_wall_count = 0;



/**********************************************
This function initialises the novint falcon controller

NoFalcons is the index of the falcon which you wish to initialise
Index 0 is first falcon.
**********************************************/
 
bool init_falcon(int NoFalcon)
{
  cout << "Setting up LibUSB" << endl;
  g_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
  if(!g_falconDevice.open(NoFalcon)) //Open falcon @ index 0  (index needed for multiple falcons, assuming only one connected)
	{
	  cout << "Failed to find falcon" << endl;
	  return false;
	}
  else
	{
	  cout << "Falcon Found" << endl;
	}

  //There's only one kind of firmware right now, so automatically set that.
  g_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>();

  //Next load the firmware to the device	
  bool skip_checksum = false;

  //See if we have firmware
  bool firmware_loaded = false;
  firmware_loaded = g_falconDevice.isFirmwareLoaded();
	
  if(!firmware_loaded)
	{
	  cout << "Loading firmware" << endl;
	  uint8_t* firmware_block;
	  long firmware_size;
	  {

		firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
		firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


		for(int i = 0; i < 20; ++i)
		  {
			if(!g_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

			  {
				cout << "Firmware loading try failed" <<endl;
			  }
			else
			  {
				firmware_loaded = true;
				break;
			  }
		  }
	  }
	}
  else if(!firmware_loaded)
	{
	  cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << endl;
	  return false;
	}
  if(!firmware_loaded || !g_falconDevice.isFirmwareLoaded())
	{
	  cout << "No firmware loaded to device, cannot continue" << endl;
	  return false;
	}
  cout << "Firmware loaded" << endl;
    
  g_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
  cout << "Homing Set" << endl;
  std::array<int, 3> forces;
  g_falconDevice.getFalconFirmware()->setForces(forces);
  g_falconDevice.runIOLoop(); //read in data      

  {
	bool stop = false;
	bool homing = false;
	bool homing_reset = false;
	usleep(100000);
	int tryLoad = 0;
	while(!stop && tryLoad < 100)
	  {
		if(!g_falconDevice.runIOLoop()) continue;
		if(!g_falconDevice.getFalconFirmware()->isHomed())
		  {
			if(!homing)
			  {
				g_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
				cout << "Falcon not currently homed. Move control all the way out then push straight all the way in." << endl;
                
			  }
			homing = true;
		  }

		if(homing && g_falconDevice.getFalconFirmware()->isHomed())
		  {
			g_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
			cout << "Falcon homed." << endl;
			homing_reset = true;
			stop = true;
		  }
		tryLoad++;
	  }
	while(!g_falconDevice.runIOLoop());
  }
  return true;    
}


// We really don't want to use this, setting the force over any sort of
// Networked connection (read anything coming from ROS is going to be very jerky
// Better for some external node to just tell us where walls are or something



// Assuming here that that we get sensors in the form
// lrtb0000 where each of the four bits tells us which sensor is turned on
void ContactensorCallback(const std_msgs::Byte& msg)
{

  bool r_sensor, l_sensor, t_sensor, b_sensor;
  r_sensor = msg.data & 0b1000000;
  l_sensor = msg.data & 0b0100000;
  t_sensor = msg.data & 0b0010000;
  b_sensor = msg.data & 0b0001000;
  

  if(r_sensor) {
	g_right_wall_pos = g_pos[1];
	g_right_wall_count = COUNT_MAX;
  }

  
  if(l_sensor) {	
	g_left_wall_pos = g_pos[1];
	g_left_wall_count = COUNT_MAX;
  }

  if(t_sensor) {
	g_top_wall_pos = g_pos[2];
	g_top_wall_count = COUNT_MAX;
  }

  if(b_sensor) {
	g_bottom_wall_pos = g_pos[2];
	g_bottom_wall_count = COUNT_MAX;
  }
	
}


int main(int argc, char* argv[])
{


  ros::init(argc,argv, "ROSfalcon");

  int baud = 115200;
  std::string port = "/dev/ttyUSB0";
  // std::string port = "/dev/ttyACM0";
	
  // port, baudrate, timeout in milliseconds
  serial::Serial arduino(port, baud, serial::Timeout::simpleTimeout(100));
  sleep(2);
  string result = arduino.read(512);
  std::cout << "found arduino, it said: " << result  << std::endl;
  
  //TODO Driver currently assumes there is only one falcon attached 
  if(init_falcon(0))
	{ 
	  cout << "Falcon Initialized Starting ROS Node" << endl;	  
	  
	  g_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
	  ros::NodeHandle node;
	  
	  ros::Rate loop_rate(30000); //HZ
	  int arduino_max_count = 0;
	  int arduino_count = 0;
	   
	  
	  std::array<double,3> forces;

	  
	  while(node.ok())
		{
		  if(g_falconDevice.runIOLoop())
			{
			  //////////////////////////////////////////////
			  //Request the current encoder positions:
			  
			  
			  //cout << "spinning in IO loop" << endl; 
			  g_pos = g_falconDevice.getPosition();

			  //std::cout << g_pos[0] << " " << g_pos[1] << " " << g_pos[2] << std::endl;
			  
			  // TODO should probably rewrite these... it's just a map taking the falcon workspace to the lynx workspace
			  g_pos[0] = ((g_pos[0] - FX_MIN)/(FX_MAX - FX_MIN) * (Z_MAX - Z_MIN) + Z_MIN);
			  g_pos[1] = ((g_pos[1] - FY_MIN)/(FY_MAX - FY_MIN) * (Y_MAX - Y_MIN) + Y_MIN);
			  g_pos[2] = ((1 - (g_pos[2] - FZ_MIN)/(FZ_MAX - FZ_MIN)) * (X_MAX - X_MIN) + X_MIN);
			  
			  //std::cout << g_pos[2] << " " << g_pos[1] << " " << g_pos[0] << std::endl;

			  
			  if (arduino_count == arduino_max_count){
				
				std::string x_cmd = std::to_string(g_pos[2]).substr(0,5);
				std::string y_cmd = std::to_string(g_pos[1]).substr(0,5);
				std::string z_cmd = std::to_string(g_pos[0]).substr(0,5);
				
				std::string cmd_string = "m,"  + x_cmd +   ","  + y_cmd +  "," + z_cmd + "!";
				//std::cout << "cmd_string" << cmd_string << std::endl;
				int bytes_written = arduino.write(cmd_string);
				std::cout << bytes_written << std::endl;
			  
				result = arduino.read(16);
				cout <<  "Bytes read: " << result.length() << ", String read: " << result << std::endl;
				//cout << "result[0] is " << result[0] << endl;
				arduino_count = 0;
				
			  }

			  // Render walls
			  forces = {0,0,0}; //xyz
			  
			  
			  if(result.length()){
				//cout << "result[0] is " << result[0] << endl;
				//TODO put this in a fcn
				bool r_sensor = result[0] == '1' ? true : false;
				bool l_sensor = result[1] == '1' ? true : false;
				bool t_sensor = result[2] == '1' ? true : false;
				bool b_sensor = result[3] == '1' ? true : false;

				//cout << r_sensor << endl;
				//	cout << t_sensor << endl;

				
				// if(r_sensor) {
				//   cout << "uh... hello" << endl;
				//   g_right_wall_pos = g_pos[1];
				//   g_right_wall_count = COUNT_MAX;
				// }
				
				// if(l_sensor) {
				//   g_left_wall_pos = g_pos[1];
				//   g_left_wall_count = COUNT_MAX;
				// }
				
				if(t_sensor && g_top_wall_count == 0) {
				  g_top_wall_pos = g_pos[2];
				  g_top_wall_count = COUNT_MAX;
				}
				
				// if(b_sensor) {
				//   g_bottom_wall_pos = g_pos[2];
				//   g_bottom_wall_count = COUNT_MAX;
				// }


				//cout << "cnt: " << g_right_wall_count << endl;
				
				//cout << "g_right_wall_pos" << g_right_wall_pos << "g_pos" << g_pos[0] << endl;
				
				// //This too
				// if (g_right_wall_count) {
				//   if(g_right_wall_pos < g_pos[1]){
				// 	cout << "adding right wall force" << endl;
				// 	forces[0] += (g_right_wall_pos - g_pos[0])*WALL_K;
				// 	g_right_wall_count--;
				// 	//cout << "forces[0] in right wall count  = " << forces[0] << endl;
				//   }
				// }
				
				// if (g_left_wall_count) {
				//   //cout << "adding left wall force" << endl;
				//   forces[0] += (g_pos[0] - g_left_wall_pos)*WALL_K;
				//   g_left_wall_count--; 
				// }

				if (g_top_wall_count) {
				  //cout << "wall count" <<  g_top_wall_count << " g_pos[2] " << g_pos[2] << " g_top_wall_pos " << g_top_wall_pos <<  endl;
				  if (g_pos[2] > g_top_wall_pos){
					//cout << "adding top wall force" << endl;
					forces[2] -= (g_top_wall_pos - g_pos[2])*10;
					//cout << "forces = " << forces[2] << endl;
				  }
				  g_top_wall_count--;
				}
				
				// if (g_bottom_wall_count) {
				//   //cout << "adding bottom wall force" << endl;
				//   forces[2] += (g_pos[2] - g_bottom_wall_pos)*WALL_K;
				//   g_bottom_wall_count--; 
				// }
				
			  } 

			  //cout << "forces:: " << forces[0] << endl;
			  
			  g_falconDevice.setForce(forces);
			  
			  //cout << "spinning" << endl;
			  //ros::spinOnce();
			  arduino_count++;
			  loop_rate.sleep();
			}
		}
	  g_falconDevice.close();
	}
  
  return 0;
}

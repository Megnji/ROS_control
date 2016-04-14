#include "ros/ros.h" 
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h"
#include <stdlib.h>     //for using the function sleep
#include <iostream>     //for using cout
#include <cmath>
using namespace std;
geometry_msgs::Twist velocityCommand;

//Because the frequency of laser callback is set, we can use int as a timer
int timer=0;

//Will set to 1 when robot detect no close obstacles
int isSwapMode=0;

//Used in swap mode, detect the longer distance of both side and choose the wider side to turn to
bool havePosAngV = false;

//Used to avoid corner
int changeDirectionCount = 0;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) 
{
  if (changeDirectionCount > 20){
    velocityCommand.angular.z = 0.0;
    changeDirectionCount = 0;
    cout << "Corner escape mode finished\n";
  }else if (changeDirectionCount > 4){
    changeDirectionCount ++;
    return;
  }else if (changeDirectionCount == 4){
    cout << "Corner escape mode on\n";
    changeDirectionCount ++;
    return;
  }
        // Example of using some of the non-range data-types
        float rangeDataNum =  1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);

	int obList[512];
        int obAtLeft = 0;
	int isAnyOb = 0; 
	int numOfOb = 0;
	float lastDistance = laserScanData->ranges[0];
        // Go through the laser data, data detection 
        for(int j = 0; j < rangeDataNum; ++j)
        {
	  float currentDistance = laserScanData->ranges[j];
	  if (abs(currentDistance- lastDistance) > 0.5 && (currentDistance < 0.9 || lastDistance<0.9)){
	    numOfOb ++;
	  }
	lastDistance = currentDistance;
	  
	  if( currentDistance < 0.3 && isfinite(currentDistance) ){
	    if (j<256){
	      obAtLeft = 0;
	    }else{
	      obAtLeft = 1;
	    }
	    isAnyOb = 1;
	 }

	  if( currentDistance < 0.2 && isfinite(currentDistance)) {
		if ( j < 256){
		  if (velocityCommand.angular.z == -0.3){
		    changeDirectionCount ++;
		  }
			velocityCommand.angular.z = 0.3;
		}else{
		  if (velocityCommand.angular.z == 0.3){
		    changeDirectionCount++;
		  }
			velocityCommand.angular.z = -0.3;
		}
		if (isSwapMode == 1) {
		  cout << "Emergency avoidence, stop swap row mode"<<"\n";}
		velocityCommand.linear.x = 0.0;
		isSwapMode = 0;
		timer = 0;

		return;
 	  }
        }
	numOfOb = (numOfOb + 1) / 2;
	//Operate the robot based on the analyzied data
	//Entering swap mode
	if (numOfOb == 0 && isSwapMode == 0){
	  float smallCount = 0;
	  float largeCount = 0;
	  for (int i=0; i<12; i++){
	    smallCount += laserScanData->ranges[i];
	    largeCount += laserScanData->ranges[511-i];
	  }
	  if (smallCount < largeCount){
	    havePosAngV = true;
	  }else{
	    havePosAngV = false;
	  }
	  isSwapMode = 1;
 	  cout << "Entering swap row mode!\n";	
	}

	//Normal avoid obstacles mode
	if(isAnyOb == 1 && isSwapMode == 0)
        {
                if (obAtLeft == 0){
		  if (velocityCommand.angular.z == -0.3){
		    changeDirectionCount ++;
		  }
                        velocityCommand.angular.z = 0.3;
                }else if (obAtLeft == 1){
		  if (velocityCommand.angular.z == 0.3){
		    changeDirectionCount++;
		  }
                        velocityCommand.angular.z = -0.3;
                }

                velocityCommand.linear.x = 0.0;
        }else if(isSwapMode == 0){  //reset the angular velocity
                velocityCommand.angular.z = 0.0;
		velocityCommand.linear.x = 0.1;

        }else{
	  timer ++;

	  //Finish the swap mode
	  if (timer > 193){ 
	    timer = 0;
	    isSwapMode = 0;
	    velocityCommand.angular.z = 0.0;
	    velocityCommand.linear.x = 0.1;
	    //Swap row mode finished 
	    cout << "Swap row mode finished!\n"; 
	  }else if (timer > 153){ // Trun 90 degrees to the right direction
		if (havePosAngV){
			velocityCommand.angular.z = 0.3;
		}else{
			velocityCommand.angular.z = -0.3;
		}
		velocityCommand.linear.x = 0.0;
	  }else if (timer > 88){   //Go straight for a while
	  	velocityCommand.linear.x = 0.1;
		velocityCommand.angular.z = 0.0;
	  }else if(timer > 38){     // keep going straight until whole body of the robot has passed the obstacle  
	    if (havePosAngV){  // then turn to right direction, based on the variable havePosAngV
			velocityCommand.angular.z = 0.3;
		}else{
			velocityCommand.angular.z = -0.3;
		}
		velocityCommand.linear.x = 0.0;
	  }else{
	    velocityCommand.angular.z = 0.0;
	    velocityCommand.linear.x = 0.1;
	  }
	}
	//Cornor avoidence status reset
	if (velocityCommand.linear.x == 0.1 && velocityCommand.angular.z == 0){
	  changeDirectionCount = 0;
	}
}

int main (int argc, char **argv) {
	// command line ROS arguments 
	ros::init(argc, argv, "pioneer_laser_node");
	
	// ROS comms access point
	ros::NodeHandle my_handle; 
	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

	// subscribe to the scan topic and define a callback function to process the data 
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
	
	// loop 10 Hz 
	ros::Rate loop_rate(10);

	// publish the velocity set in the call back 
	while(ros::ok()) 
	{
		ros::spinOnce(); 
		loop_rate.sleep();
		
		// publish 
		vel_pub_object.publish(velocityCommand);
	} 
	return 0;
}

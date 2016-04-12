#include "ros/ros.h" 
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h"
#include <stdlib.h>     //for using the function sleep
#include <iostream>     //for using cout
using namespace std;
geometry_msgs::Twist velocityCommand;
int timer=0;
int isSwapMode=0;
bool havePosAngV = false;
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) 
{
        // Example of using some of the non-range data-types
        float rangeDataNum =  1 + (laserScanData->angle_max - laserScanData->angle_min)/(laserScanData->angle_increment);

        velocityCommand.linear.x = 0.1;
	int obList[512];
	int obSize = 0;
        int obAtLeft = 0;
	int isAnyOb = 0; 
	int numOfOb = 0;
	float lastDistance = laserScanData->ranges[0];
        // Go through the laser data 
        for(int j = 0; j < rangeDataNum; ++j)
        {
	  float currentDistance = laserScanData->ranges[j];
	  if (abs(currentDistance- lastDistance) > 0.1 && (currentDistance < 0.1 || lastDistance<0.1)){
		numOfOb ++;
		
	  }
	  lastDistance = currentDistance;
	  cout << currentDistance << "   ";
	  if( laserScanData->ranges[j] < 0.10 ){
	    if (j<256){
	      obAtLeft = 0;
	    }else{
	      obAtLeft = 1;
	    }
	    isAnyOb = 1;
	    obSize ++;
	  }

	  if( laserScanData->ranges[j]< 0.05) {
		if ( j < 256){
			velocityCommand.angular.z = 0.3;
		}else{
			velocityCommand.angular.z = -0.3;
		}
		velocityCommand.linear.x = 0;
		isSwapMode = 0;
		cout << "Emergency avoidence mode"<<"\n";
		return;
 	  }
        }
	numOfOb = (numOfOb + 1) / 2;
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
 	  cout << "Entering swap line mode!" << "\n";	
	}
	if(isAnyOb == 1 && isSwapMode == 0)
        {
                if (obAtLeft == 0){
                        velocityCommand.angular.z = 0.3;
                }else if (obAtLeft == 1){
                        velocityCommand.angular.z = -0.3;
                }

                velocityCommand.linear.x = 0;
        }else if(isSwapMode == 0){
                velocityCommand.angular.z = 0;

        }else{
	  timer ++;
	  //Finish the swap mode
	  if (timer > 195){
	    timer = 0;
	    isSwapMode = 0;
	  }else if (timer > 145){
		if (havePosAngV){
			velocityCommand.angular.z = 0.3;
		}else{
			velocityCommand.angular.z = -0.3;
		}
		velocityCommand.linear.x = 0.0;
	  }else if (timer > 90){
	  	velocityCommand.linear.x = 0.1;
		velocityCommand.angular.z = 0.0;
	  }else if(timer > 38){  
		if (havePosAngV){
			velocityCommand.angular.z = 0.3;
		}else{
			velocityCommand.angular.z = -0.3;
		}
		velocityCommand.linear.x = 0.0;
	  }
	}
	cout << "\n----------------------\n";
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

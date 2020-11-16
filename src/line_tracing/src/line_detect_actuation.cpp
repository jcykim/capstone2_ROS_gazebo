#include <ros/ros.h>
#include "core_msgs/line_segments.h"
#include "core_msgs/yolomsg.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <std_msgs/Float64.h>

#include <ros/package.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <math.h>
#include <boost/thread.hpp>
#include <sys/types.h>
#include <sys/socket.h>

#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>

using namespace std;

ros::Publisher pub_right_front_wheel;
ros::Publisher pub_left_front_wheel;
ros::Publisher pub_left_rear_wheel;
ros::Publisher pub_right_rear_wheel;

std_msgs::Float64 right_front;
std_msgs::Float64 left_front;
std_msgs::Float64 left_rear;
std_msgs::Float64 right_rear;

boost::mutex map_mutex;
int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

float leftt;
float rightt;
float front;
float behind;

float fr;
float fl;


bool detect_fl;
bool detect_fr;
int vibration = 0;
int min_index;
int left_wpn;
int right_wpn;
int wpn_idx;

bool detect_frontg;
bool nodetect_frontg;
bool detect_frg;
bool detect_flg;
bool sameg;
bool largeg;


int correct = 0;
int starttemp = 0;
int yolostop = 0;
bool isenabled = true;
bool parkingmode = false; // only parking = true
bool rotate_mode = false;
bool firstpark = true;
bool secondpark = false;
bool isyolofinished= false;
int temp1 = 0;

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{	
	front = 3.5;
	behind = 3.5;
	leftt = 3.5;
	rightt = 3.5;
	fl = 3.5;
	fr = 3.5;
	min_index = 0;

	detect_frontg = false;
	detect_frg = false;
	detect_flg = false;
	largeg = false;
	sameg = false;
	nodetect_frontg = false ;


	map_mutex.lock();

	int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
	// std::cout << "-------------------------------------------------------"<<std::endl;
    for(int i = 0; i < count; i++)
    {	
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
		if(isinf(lidar_distance[i])){
			lidar_distance[i] = 3.5;
		}else{
			if (count > 105)
			{
				if (lidar_distance[count - 75] - lidar_distance[count - 105] < 0.05 && lidar_distance[count - 75] - lidar_distance[count - 105] >=-0.05) {
					sameg=true;
				}
				if (lidar_distance[count-75] < lidar_distance[count - 105]){
					largeg = true;
				}
			} 
			if(i ==0 || i == 10 || i == count-10 ){
				if(front>lidar_distance[i]){
					front = lidar_distance[i];
					min_index = i;
				}
			}else if(i == 25 || i == 35 || i == 45){
				if(fl>lidar_distance[i]){
					fl = lidar_distance[i];
				}
			}else if(i == 75 || i == 90 || i == 105){
				if(leftt>lidar_distance[i]){
					leftt = lidar_distance[i];
				}
			}else if(i == 180 || i == 165 || i == 150 || i == 195 || i == 210){
				if(behind>lidar_distance[i]){
					behind = lidar_distance[i];
				}
			}else if(i == count-75 || i == count-90 || i == count-105){
				if(rightt>lidar_distance[i]){
					rightt = lidar_distance[i];
				}
			}else if(i == count - 25 || i == count - 35 || i == count - 45){
				if(fr>lidar_distance[i]){
					fr = lidar_distance[i];
				}
			}
		}	
    }

	if(front<0.67){
		detect_frontg = true;
	}
  	if (front > 1.2 ){
		nodetect_frontg = true;
	}

	map_mutex.unlock();

}

void yolocallback(const core_msgs::yolomsg::ConstPtr& msg){
	int num1 = msg -> num;

	if (num1 == 1 && isyolofinished) {
		string name1 = msg -> className[0];

		if ( name1 == "clock" || name1 == "bird"){
			parkingmode = true;
		}
	}

	return;
}
void yolocallback2(const core_msgs::yolomsg::ConstPtr& msg){
	int num1 = msg -> num;
	if (num1 == 0 ) {
		yolostop = 0;	
		return;
	}
	if (num1 == 1) {
		string name1 = msg -> className[0];
		if ( name1 == "stop sign" ) {
			yolostop = 0;
			cout << "22" << endl;
			return;
		}
	}
	if (num1 > 1) {
		cout << "33" << endl;
		yolostop = 1;
	}
	return;
}
void decision_center(const core_msgs::line_segments::ConstPtr& msg) {
    if (!isenabled) return;
    int count; //this is the number of actual data points received
    int array_size = msg->size;

    if (array_size == 0)
        return;
 	if (starttemp > 250){
		cout << "dddd" << endl;
		ros::param::set("/yolo1_finished", true);
	}
    count = array_size;
    for (int i =0;i < array_size;i++)
    {
        if (isnan(msg->com_x[i]))
        {
            count=i;
            //cout << "count = " << count << "    " << msg->com_x[0] << endl;
            break;
        }
    }

   if (yolostop  == 1 && temp1 < 6) {
        right_front.data=-49;
        left_front.data=-49;
        left_rear.data=-49;
        right_rear.data=-49;
	temp1 ++;
	cout << "44" << endl;
	}
   else if (yolostop == 1 && temp1 >= 6){
	 right_front.data=0;
        left_front.data=0;
        left_rear.data=0;
        right_rear.data=0;
		cout << "99" << endl;
	}
   else if (starttemp < 70 ) {
        right_front.data=50;
        left_front.data=50;
        left_rear.data=50;
        right_rear.data=50;
	cout << "55" << endl;
	}
   
   else if (count>0){
        double error = msg->com_x[0];
        right_front.data=50;
        left_front.data=50;
        left_rear.data=50;
        right_rear.data=50;
	cout << "66" << endl;

        if (abs(error)<55){
            correct = 0;
        }

        if (abs(error)>80 || correct==1){
            correct =1;
            if (error>0){
                left_front.data = 50; //50
                right_front.data = -45;//-48
                left_rear.data = 30; //20
                right_rear.data = -30; //-20
            }
            else{
                right_front.data = 50;
                left_front.data = -45;
                right_rear.data = 30;//chaznge
                left_rear.data = -30;
            }
        }

    }
    if (parkingmode == true){
	if (detect_frontg == false && !rotate_mode && firstpark){
		right_front.data=50;
		left_front.data=50;
		left_rear.data=50;
		right_rear.data=50;
		cout << "first parking" << endl;
	}
	else if (largeg && sameg == false && !rotate_mode) {
		right_front.data=50;
		left_front.data=-50;
		left_rear.data=-50;
		right_rear.data=50;
		cout << "first rotation" << endl;
		firstpark = false;
	}
	else if (!largeg && sameg == false && !rotate_mode) {
		right_front.data=-50;
		left_front.data=50;
		left_rear.data=50;
		right_rear.data=-50;
		cout << "first rotation" << endl;
		firstpark = false;
	}
	else if (sameg == true&& !rotate_mode) {
		secondpark = true;
	}
	if (secondpark == true)  {
		right_front.data=50;
		left_front.data=50;
		left_rear.data=50;
		right_rear.data=50;
		cout << "second parking" << endl;
		if (detect_frontg == true){
			right_front.data=0;
			left_front.data=0;
			left_rear.data=0;
			right_rear.data=0;
			secondpark = false;
			rotate_mode = true;
		}
	}
	if (rotate_mode == true){
		if (nodetect_frontg == false){
			secondpark = false;
			cout << "doing last rotation" << endl;
			right_front.data=50;
			left_front.data=-40;
			left_rear.data=-40;
			right_rear.data=50;
		}else {
			secondpark = false;
			right_front.data=0;
			left_front.data=0;
			left_rear.data=0;
			right_rear.data=0;
			cout << "parking finisih" << endl;
			ros::param::set("/entrance_finished", true);
		}
	}

	
    }
        pub_left_front_wheel.publish(left_front);
        pub_right_front_wheel.publish(right_front);
        pub_left_rear_wheel.publish(left_rear);
        pub_right_rear_wheel.publish(right_rear);

        cout << left_front.data << "    " << right_front.data << "    " << right_rear.data<< "    " << left_rear.data << endl;

    
}

int main (int argc, char **argv) 
{
    ros::init (argc, argv, "line_detect_actuation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/segments", 10, decision_center);
    ros::Subscriber sub2 = nh.subscribe("/yolo_result",100, yolocallback);
    ros::Subscriber sub4 = nh.subscribe("/yolo_result2",100, yolocallback2);
    ros::Subscriber sub3 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);

   
    pub_right_front_wheel = nh.advertise<std_msgs::Float64>("model20/right_front_wheel_velocity_controller/command", 10);
    pub_left_front_wheel = nh.advertise<std_msgs::Float64>("model20/left_front_wheel_velocity_controller/command", 10);
    pub_right_rear_wheel = nh.advertise<std_msgs::Float64>("model20/right_rear_wheel_velocity_controller/command", 10);
    pub_left_rear_wheel = nh.advertise<std_msgs::Float64>("model20/left_rear_wheel_velocity_controller/command", 10);
     
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        bool entrance_finished;
	bool yolo1_finished;
	nh.getParam("/yolo1_finished", yolo1_finished);
        if (isenabled && nh.getParam("/entrance_finished", entrance_finished)) {
	 
	    starttemp++;

	     if (starttemp%10 == 0) cout << starttemp << endl;
	if( yolo1_finished ){
		isyolofinished = true;
	}
            if (entrance_finished) {
                isenabled = false;
            }
        }

        loop_rate.sleep();
    }
    return 0;
}

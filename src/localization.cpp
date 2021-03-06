#include "ros/ros.h"
#include <ros/console.h> 
#include <stdlib.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <iostream>
#include <vector>
#include <string>
//#include <math.h>       /* cos */

//#include <multi_ardrone_sim/posevector.h>

using namespace std;

#define PI 3.14159265

class localizer{

private:

	geometry_msgs::Pose Pose;
			
	ros::NodeHandle nh;
	ros::Subscriber PoseSub;
	ros::Publisher  PosePub;

	string ns;
	int index=0; // model index on gazebo
	bool know_index=false;
public:

	localizer(){
		ns=ros::this_node::getNamespace();
		ns.erase(0,1);
		//cout<<"Getting ns: "<<ns<<endl;
		PoseSub = nh.subscribe("/gazebo/model_states", 10, &localizer::getIndex, this);
		PosePub = nh.advertise<geometry_msgs::Pose>("cfpose", 10, this);
	}

	void getIndex(const gazebo_msgs::ModelStates::ConstPtr& msg){
		string str;
		//cout<<know_index<<endl;
		if (!know_index){
			int no_states = msg->name.size()-1;
			int i(0);
			//cout<<know_index<<endl;
			while (!know_index){

				// cout<<msg->name[i]<<endl;
				str="/"+msg->name[i];
				//cout<<ns<<" "<<str<<endl;
				if (ns==str){
					//cout<<" found ns "<<endl;
					index=i;  know_index=true;
				}
				else{i++;}
			}
		}
		else{
			//cout<<"Gonna publish the pose"<<endl;
			Pose=msg->pose[index];
			PosePub.publish(Pose);
		}
	}
};

int main(int argc, char** argv){

	ros::init(argc, argv, "getPose");	
	localizer cfpose;

	ros::Rate r(50);
	while(ros::ok()){
		
		ros::spinOnce();
		r.sleep();		
	}
}
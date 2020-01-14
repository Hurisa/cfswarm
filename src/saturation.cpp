#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <string>
#include <math.h>  
#include <stdlib.h>
#include <numeric>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cfswarm/getAverageMapValue.h>
#include <algorithm>
#define PI 3.14159265
using namespace std;

class Saturator{

private:
	ros::NodeHandle nh;
	double tr,rdot,rdelta,I0,S;
	double Sfun;
	ros::Subscriber brightSub;
	ros::Publisher  saturationPub;
	ros::Publisher  rStatePub;
	
	double ns,ts,n;
	string nspace;
public:
	Saturator(){
		nspace = ros::this_node::getNamespace();
		brightSub=nh.subscribe("brightness",1,&Saturator::updateSaturation, this);
		saturationPub=nh.advertise<std_msgs::Float32>("saturation",1,this);	
		rStatePub=nh.advertise<std_msgs::Float32>("rstate",1,this);	
		rdelta=0.01;
		tr=0.45;	
		rdot=0;
		
		nh.getParam("/Sfun", Sfun);
		//Sfun=1;

		/// Parameters for saturation functions.
	 	ns=15; ts=.5; n=100;
	}


	void updateSaturation(const std_msgs::Float32& msg){

		if(msg.data>tr){
			rdot=min(rdot+rdelta,1.0);
		}
		else{
			rdot=max(0.0,rdot-rdelta);
		}

		if (Sfun==1.0){
			S=exp(ns*(rdot-ts))/(exp(ns*(rdot-ts))+1);
			cout<<nspace<<" "<<S<<" "<<rdot<<endl;
		}
		else if(Sfun==-1.0){
			S=1;
		}
		else{
			S=0;
		}


		std_msgs::Float32 smsg;
		smsg.data=S;
		saturationPub.publish(smsg);

		std_msgs::Float32 rmsg;
		rmsg.data=rdot;
		rStatePub.publish(rmsg);

	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "saturation");
	Saturator saturator;
	ros::Rate r(1);
	while (ros::ok()){
		
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
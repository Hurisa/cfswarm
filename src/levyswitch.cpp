#include "ros/ros.h"
#include <ros/console.h> 
#include <stdlib.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cfswarm/getAverageMapValue.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <vector>
#include <string>
#include <math.h>       /* cos */
     

using namespace std;

#define PI 3.14159265

class Endocrine{

private:
	ros::Subscriber PoseSub, ReorientSub, ResetSub;
	ros::Publisher  PubMu, PubBeta, PubPts;

	geometry_msgs::Pose Pose;
	ros::ServiceClient hormone_client;               //Update_params service client
	cfswarm::getAverageMapValue hormone_srv;
	string ns;
    ros::NodeHandle nh;
	// Hormone parameters
	float rho; // wrapped Cauchy parameter
	float a0,a1,a2,muSettle,b1,b2,scale;
	float mu,mupast,beta,betapast;	
	float f,tau,C,b;

	bool state;

public:

	Endocrine(){
		
		PoseSub 		= nh.subscribe("cfpose"	, 10, &Endocrine::updatePose, this);
		ReorientSub 	= nh.subscribe("state"	, 10, &Endocrine::CheckState, this);
		ResetSub		= nh.subscribe("resetMp", 10, &Endocrine::resetHormones,this);
		PubMu			= nh.advertise<std_msgs::Float32>("cfmu"	,10, this);
		PubBeta 		= nh.advertise<std_msgs::Float32>("cfbeta"	,10, this);
		PubPts	 		= nh.advertise<std_msgs::Float32>("points"	,10, this);
		
		ns				= ros::this_node::getNamespace();
		hormone_client	=nh.serviceClient<cfswarm::getAverageMapValue>("/getAverageMapValue");
		
		nh.getParam("/mu", muSettle);nh.getParam("/rho", rho);
		nh.getParam("/muSettle", muSettle); nh.getParam("/a1", a1); nh.getParam("/a2", a2);
		nh.getParam("/b1", b1); nh.getParam("/b2", b2); nh.getParam("/scale", scale);
		a0=muSettle*(1-a1);
		// Adaptive Collective Levy Walk variables
		tau=0.0;
		C=50;
		b=0.04;

		f=0;//fpast=0;beta=0;betapast=0;
	}

	void resetHormones(const std_msgs::Bool& msg){
		if(msg.data){
			mu=muSettle;
			f=0;//fpast=0;beta=0;betapast=0;
		}
	}

	void CheckState(const std_msgs::Bool& msg){
		state=msg.data;
	}

	void updatePose(const geometry_msgs::Pose& msg){
		Pose=msg;	
	}	

	void Hormone(){
		//cout<<"---------------------------------"<<endl;
		std_msgs::Float32 msgMu, msgBeta, points;
		// if(!state){
		hormone_srv.request.posX=Pose.position.x;
		hormone_srv.request.posY=Pose.position.y;
		hormone_client.call(hormone_srv);
		//fpast=f; betapast=beta; mupast=mu;
		f=hormone_srv.response.S;
		

		if (f>0){
			mu=3.0;
			tau=0.0;
		}else{
			tau=tau+1;
			mu=1+max((float)1.0,erfc(b*(tau-C)));

		}
		//cout<<mu<<" "<<tau<<endl;

		//mu=min(a0+a1*mupast+a2*stimulus,float(3.0));
		beta=0;
		// }
		msgMu.data=mu, msgBeta.data=beta, points.data=f;
		PubMu.publish(msgMu);
		PubBeta.publish(msgBeta);
		PubPts.publish(points);
	}
};

int main(int argc, char** argv){

	ros::init(argc, argv, "LevySwitch");	

	Endocrine aes;

	ros::Rate r(10);
	while(ros::ok()){
		aes.Hormone();
		ros::spinOnce();
		r.sleep();		
	}
}
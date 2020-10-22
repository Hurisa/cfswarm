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

class LevyBoid{

private:

	float theta, jump, CurrentDistance;
	
	geometry_msgs::Pose Pose;
	geometry_msgs::Twist v;
	std_msgs::Bool state;
	tf2::Quaternion Q;	
	tf2::Matrix3x3 Rotation;
	double roll, pitch, yaw;
	float w;
	ros::NodeHandle nh;
	ros::Subscriber PoseSub, adaptMuSub, adaptBetaSub,truncateSub;
	ros::Publisher PubVel, PubState, PubJump;

	// ros::ServiceClient hormone_client;               //Update_params service client
	// cfswarm::getAverageMapValue hormone_srv;

	bool changeTheta, truncation;

	string ns;
	
	float mu, beta, rho, scale, muSettle; 

public:

	LevyBoid(){

		PoseSub 		= nh.subscribe("cfpose"	, 10, &LevyBoid::getCurrentDistance, this);
		adaptMuSub 		= nh.subscribe("cfmu"	, 10, &LevyBoid::UpdateMu, this);
		adaptBetaSub 	= nh.subscribe("cfbeta"	, 10, &LevyBoid::UpdateBeta, this);
		truncateSub		= nh.subscribe("truncateWalk", 10, &LevyBoid::UpdateTruncation, this);


		PubVel	= nh.advertise<geometry_msgs::Twist>("levyVel",10, this);
		PubJump	= nh.advertise<std_msgs::Float32>("jump",10, this);
		PubState= nh.advertise<std_msgs::Bool>("state",10, this);
		ns	= ros::this_node::getNamespace();
		nh.getParam("/mu", muSettle);nh.getParam("/rho", rho);nh.getParam("/scale", scale);

		// CurrentDistance=0.0;
		// jump=0.0;
		// beta=0.0;
		// changeTheta=false;
		genJump();


	}	
	void UpdateTruncation(const std_msgs::Bool msg){
		truncation=msg.data;
	}

	void UpdateMu(const std_msgs::Float32 msg){
		if(!changeTheta){
		mu=msg.data;
		}
	}

	void UpdateBeta(const std_msgs::Float32 msg){
		if(!changeTheta){
		beta=msg.data;
		jump=(1-beta)*jump;
		}
	}

	void getCurrentDistance(const geometry_msgs::Pose& msg){
		geometry_msgs::Pose LastPose=Pose;
		if (!isnan(msg.position.x)){Pose=msg;}	  
		if (!changeTheta){
		CurrentDistance+=sqrt(pow(Pose.position.x-LastPose.position.x,2)+pow(Pose.position.y-LastPose.position.y,2));
		}	
		checkJump();
 		ComputeVel();
	}	

	void checkJump(){
		//cout<<" Jump at levy function "<<CurrentDistance<<" "<<jump<<endl;
		if (CurrentDistance>=jump||isnan(jump))
		{			
			std_msgs::Float32 jumpMsg;
			jumpMsg.data=CurrentDistance;
			PubJump.publish(jumpMsg);
			genJump();
		}
		else if(CurrentDistance<jump && truncation){
			std_msgs::Float32 jumpMsg;
			if (CurrentDistance>10e-4){
				jumpMsg.data=CurrentDistance;
				PubJump.publish(jumpMsg);
				CurrentDistance=0;
			}

		}
	}

	void genJump(){

			CurrentDistance=0;
			struct timeval time;
			gettimeofday(&time,NULL);
			srand((time.tv_sec * 1000) + (time.tv_usec / 1000));

			double sigma=10.0;
			double muOne=mu-1;
			double muTwo=2-mu;
			double x, y;
			double U1, U2, U3, phi, r;

			U1=double(rand())/double(RAND_MAX)*2-1;
	        U2=double(rand())/double(RAND_MAX)*2-1;
	        U3=double(rand())/double(RAND_MAX)*2-1;

			U1 = U1*(PI/2);	U2 = (U2+1)/2; phi = drawOrientation();

		    r = (sin(muOne * U1) / pow(cos(U1), 1/muOne) ) * pow((cos(muTwo * U1) / U2), (muTwo / muOne));
		    x = r * cos(phi); y = r * sin(phi);
		    //cout<<"Jump: "<<jump<<"scale "<<scale<<endl;
		    theta=atan2(y,x); jump=scale*sqrt(pow(x,2)+pow(y,2));

	        changeTheta=true;
	        state.data=changeTheta;
	        PubState.publish(state);


	}

	double drawOrientation(){

		double t, phi;		    
            t=double(rand())/double(RAND_MAX);
            phi=-2*atan((tan(PI*t)*(rho - 1))/(rho + 1));
            if (phi<-PI){
                phi=phi+2*PI;              
            }
            else if (phi>PI){
                phi=phi-2*PI;
            }          
         return phi;
	}

	void ComputeVel(){

		tf2::fromMsg(Pose.orientation, Q);
 		Rotation.setRotation(Q);	
 		Rotation.getEulerYPR(yaw, pitch, roll);

		if (theta<0){theta+=2*PI;}
		if (yaw<0){yaw+=2*PI;}
		//cout<<"abs(theta-yaw) "<<abs(theta-yaw)<<endl;
		if (abs(theta-yaw)>0.1 && changeTheta){
			w=(theta-yaw);
			if(abs(theta-yaw)>PI){w=-w;}
		}
		else{
			w=0;
			changeTheta=false;
			state.data=changeTheta;
	        
		}
		v.linear.x=0.0; 	v.linear.y=0.0; 	v.linear.z=0.0;
		v.angular.x=0.0; 	v.angular.y=0.0; 	v.angular.z=w;	
		//cout<<"namespace"<< ns << " levy angular "<<w<<endl;
		PubVel.publish(v);
		PubState.publish(state);


	}	
};

int main(int argc, char** argv){

	ros::init(argc, argv, "Levy");	

	LevyBoid boid;

	ros::Rate r(10);
	while(ros::ok()){

		ros::spinOnce();
		r.sleep();		
	}
}
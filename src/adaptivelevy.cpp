#include "ros/ros.h"
#include <ros/console.h> 
#include <stdlib.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
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
	tf2::Quaternion Q;	
	tf2::Matrix3x3 Rotation;
	double roll, pitch, yaw;
	float w;
	ros::NodeHandle nh;
	ros::Subscriber PoseSub;
	ros::Publisher PubVel;

	ros::ServiceClient hormone_client;               //Update_params service client
	cfswarm::getAverageMapValue hormone_srv;

	bool changeTheta;

	string ns;

	// Hormone parameters
	float rho; // wrapped Cauchy parameter
	float a0,a1,a2,muSettle,b1,b2;
	float miu,miupast,beta,betapast;	
	float f,fpast,stimulus,bstimulus;

public:

	LevyBoid(){

		PoseSub = nh.subscribe("cfpose", 10, &LevyBoid::getCurrentDistance, this);
		PubVel	= nh.advertise<geometry_msgs::Twist>("levyVel",10, this);
		CurrentDistance=0;
		jump=0;
		changeTheta=false;
		ns	= ros::this_node::getNamespace();
		hormone_client=nh.serviceClient<cfswarm::getAverageMapValue>("/getAverageMapValue");
		nh.getParam("/miu", miu);nh.getParam("/rho", rho);

		nh.getParam("/muSettle", muSettle); nh.getParam("/a1", a1); nh.getParam("/a2", a2);
		nh.getParam("/b1", b1); nh.getParam("/b2", b2);
		a0=muSettle*(1-a1);

		f=0;fpast=0;beta=0;betapast=0;
	}	

	void getCurrentDistance(const geometry_msgs::Pose& msg){
		geometry_msgs::Pose LastPose=Pose;

		if (!isnan(msg.position.x)){Pose=msg;}
	  
		if (!changeTheta){
		CurrentDistance+=sqrt(pow(Pose.position.x-LastPose.position.x,2)+pow(Pose.position.y-LastPose.position.y,2));
		}
		Hormone();
		checkJump();
 		ComputeVel();
	}	

	void checkJump(){

		if (CurrentDistance>=jump)
		{
			CurrentDistance=0;
			struct timeval time;
		    gettimeofday(&time,NULL);
		    srand((time.tv_sec * 1000) + (time.tv_usec / 1000));

		    double mu=miu;
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

	        theta=atan2(y,x); jump=sqrt(pow(x,2)+pow(y,2));
	 
	        changeTheta=true;
		}
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
		}
		v.linear.x=0.0; 	v.linear.y=0.0; 	v.linear.z=0.0;
		v.angular.x=0.0; 	v.angular.y=0.0; 	v.angular.z=w;	
		//cout<<"namespace"<< ns << " levy angular "<<w<<endl;
		PubVel.publish(v);
	}	

	void Hormone(){
		cout<<"---------------------------------"<<endl;
		if(!changeTheta){
		hormone_srv.request.posX=int(round(Pose.position.x));
		hormone_srv.request.posY=int(round(Pose.position.y));
		hormone_client.call(hormone_srv);
		fpast=f; betapast=beta; miupast=miu;
		f=hormone_srv.response.S;

		if (f-fpast>0){stimulus=1;}else{stimulus=0;}
		if (f-fpast>=0){bstimulus=0;}else{bstimulus=1*(3-miu)/(3-muSettle);}

		miu=min(a0+a1*miupast+a2*stimulus,float(3.0));
		beta=min(b1*betapast+b2*bstimulus,float(1.0));
		cout<<"Original Jump: "<<jump<<endl;
		jump=(1-beta)*jump;	
		cout<<"updated Jump: "<<jump<<endl;
		}

		
		cout<<"mupast: "<<miupast<<" |betapast: "<<betapast<<endl;
		cout<<"CurrentDistance: "<<CurrentDistance<<" |jump: "<<jump<<endl;
		cout<<"mu: "<<miu<<" |beta: "<<beta<<endl;
		cout<<"---------------------------------"<<endl;
	}
};

int main(int argc, char** argv){

	ros::init(argc, argv, "Levy");	

	LevyBoid boid;

	ros::Rate r(1);
	while(ros::ok()){

		ros::spinOnce();
		r.sleep();		
	}
}
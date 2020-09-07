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
#include <cfswarm/posevector.h>
#include <cfswarm/floatvector.h>
#define PI 3.14159265
using namespace std;

class Firefly{

private:
	double I0, Imax, Xpos, Ypos, points;
	double beginTime, currentTime;
	geometry_msgs::Pose Pose;


	ros::Subscriber poseSub,PtsSub;
	ros::Subscriber brightSub;

	ros::Publisher  velToLightPub;
	ros::Publisher  brightnessPub;
	

	ros::ServiceClient update_brightness_client;               //Update_params service client
	cfswarm::getAverageMapValue brightness_srv;
	
	string decay;
public:
	Firefly(ros::NodeHandle nh, string ns, double swarmSize){

		nh.getParam("/decay",decay);

		poseSub=nh.subscribe(ns+"/cfpose",10,&Firefly::updatePose, this);
		// cout<<"decay: "<<decay<<" "<<endl;
		// cout<<"is linear? "<<decay.compare("linear")<<" "<<endl;
		if(decay.compare("linear")==0){
			cout<<"linear decay"<<endl;
			PtsSub=nh.subscribe(ns+"/points",10,&Firefly::updateI0, this);
		}else{
			PtsSub=nh.subscribe(ns+"/cfbeta",10,&Firefly::updateI0mu, this);
		}
		//brightSub=nh.subscribe(ns+"/brightness",10,&Firefly::updateBrightness, this);
		velToLightPub=nh.advertise<geometry_msgs::Twist>(ns+"/fireVel",10,this);
		brightnessPub=nh.advertise<std_msgs::Float32>(ns+"/brightness",10,this);
		ros::Time time=ros::Time::now();
		beginTime=time.toSec();

		update_brightness_client=nh.serviceClient<cfswarm::getAverageMapValue>("/getAverageMapValue");
		I0=0.0;
		Imax=0.0;
		
	}
	void updatePose(const geometry_msgs::Pose& msg){
		 Xpos=msg.position.x;
		 Ypos=msg.position.y;

		 Pose=msg;
	}
	
	void updateI0mu(const std_msgs::Float32& msg){
		I0=msg.data;
	}


	void updateI0(const std_msgs::Float32& msg){

		points=msg.data;
		if (points>0){
			ros::Time time=ros::Time::now();
			currentTime=time.toSec();
			if(currentTime-beginTime<10e-6){
				// I0=0;
				// Imax=0;
			}else{
			I0=I0+points/(currentTime-beginTime);
		;
			}
			beginTime=currentTime;
		}
		else{
			
			I0=max(I0-1,0.0);
		}

		std_msgs::Float32 brightmsg;
		brightmsg.data=I0;
		brightnessPub.publish(brightmsg);

	}

	double getX(){
		return Xpos;
	}

	double getY(){
		return Ypos;
	}

	geometry_msgs::Pose getPose(){
		return Pose;
	}

	double getBrightness(){
		return I0;
	}

	void publishVel(geometry_msgs::Twist v){

 		velToLightPub.publish(v);
	}
};


void attraction (vector<Firefly*> fireflies, double threshold, double gamma){

double x,y,xf,yf,vx,vy, sumI;
double lightAngle,d,beta,alpha;
tf2::Quaternion Q; tf2::Matrix3x3 Rotation;
geometry_msgs::Twist v;


	for(int i=0;i<fireflies.size();i++){
		x=fireflies[i]->getX(); y=fireflies[i]->getY();

		tf2::fromMsg(fireflies[i]->getPose().orientation, Q);
		Rotation.setRotation(Q);
				
		lightAngle=0.0;
		sumI=0.0;
		for(int j=0;j<fireflies.size();j++){
			xf=fireflies[j]->getX(); yf=fireflies[j]->getY();
			d=sqrt(pow(x-xf,2)+pow(y-yf,2));

			if (d<threshold && i!=j){
		
				if (fireflies[j]->getBrightness()>fireflies[i]->getBrightness()){								

					tf2::Vector3 c(xf-x,yf-y,0);
				
					vx = (Rotation.tdotx(c));
					vy = (Rotation.tdoty(c));
					alpha=atan2(vy,vx);
			
					//compute attractivness
					beta=exp(-gamma*pow(d,2));
					//compute turning angle
					lightAngle=lightAngle+beta*alpha;
					sumI=sumI+beta;
					
					//cout<<fireflies[i]->getBrightness()<<" "<<fireflies[j]->getBrightness()<<endl;
				}
				else{
					lightAngle=lightAngle;
				}

			}
		}
		
		if (sumI>0.0){		
			lightAngle=lightAngle/sumI;	
		}

		if (lightAngle<-PI ){
			lightAngle=lightAngle+2*PI; 	
		}                                             
        else if (lightAngle>PI){
        	lightAngle=lightAngle-2*PI;
        }            
        //cout<<i<<" "<<lightAngle<<endl;
        v.linear.x=0.0; 	v.linear.y=0.0; 	v.linear.z=0.0;
        v.angular.x=0.0;  v.angular.y=0.0;  v.angular.z=lightAngle;
		fireflies[i]->publishVel(v);
		//cout<<"----------------------"<<endl;
                    
	}
	

}

int main(int argc, char** argv)
{
	/* code */
	ros::init(argc, argv, "firefly");
	ros::NodeHandle nh;

    double threshold, gamma;
	vector<string> namespaces;

	ros::Publisher  SwarmPosePub = nh.advertise<cfswarm::posevector>("/SwarmPose",10);
	ros::Publisher  SwarmBrightnessPub = nh.advertise<cfswarm::floatvector>("/SwarmBrightness",10);

	cfswarm::posevector 	PoseVector;
	cfswarm::floatvector 	BrightnessVector;
	
	nh.getParam("/namespaces", namespaces);
	nh.getParam("/threshold", threshold);
	nh.getParam("/gamma", gamma);

	PoseVector.poses.resize(namespaces.size());
	BrightnessVector.values.resize(namespaces.size());
	vector<Firefly*> fireflies;
	for (int i = 0; i < namespaces.size(); i++){
		fireflies.push_back(new Firefly(nh, namespaces[i], namespaces.size()));		
	}
	


	ros::Rate r(10);

	while (ros::ok()){
		//cout<<"threshold "<<threshold<<endl;
		attraction(fireflies, threshold, gamma);

		for (int i = 0; i < namespaces.size(); i++)
		{
			PoseVector.poses[i]=fireflies[i]->getPose();
			BrightnessVector.values[i].data=fireflies[i]->getBrightness();
		}
		SwarmPosePub.publish(PoseVector);
		SwarmBrightnessPub.publish(BrightnessVector);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include <math.h>  
#include <stdlib.h>
#include <numeric>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class Avoidance{

private:
	ros::Subscriber poseSub;
	ros::Publisher	avoidVel;
	geometry_msgs::Pose Pose;
	vector<double> distances;
	tf2::Matrix3x3 Rotation;
	tf2::Quaternion Q;
	double roll,pitch,yaw;

	double radius;
	
public:

	Avoidance(ros::NodeHandle nh, string ns, double swarmSize){
		poseSub=nh.subscribe(ns+"/cfpose",10,&Avoidance::updatePose, this);
		avoidVel=nh.advertise<geometry_msgs::Twist>(ns+"/avoidVel",10,this);
		//distances.resize(swarmSize);	
	}

	void updatePose(const geometry_msgs::Pose msg){
		Pose=msg;
		tf2::fromMsg(Pose.orientation, Q);
 		Rotation.setRotation(Q);	
 		Rotation.getEulerYPR(yaw, pitch, roll);
	}

	void PublishMsg(geometry_msgs::Twist p){
		avoidVel.publish(p);
	}

	geometry_msgs::Pose getPose(){
		return Pose;
	}
	tf2::Matrix3x3 getRotMatrix(){
		return Rotation;
	}
};

double getDistance(geometry_msgs::Pose p, geometry_msgs::Pose q){
	
	double r;
	r=sqrt(pow(p.position.x-q.position.x,2)+pow(p.position.y-q.position.y,2));
	return r;
}



void ComputeVel(vector<Avoidance*> avoid, double d){

double dist;
vector<double> Xc_avoid, Yc_avoid;
float XSep, YSep;
float Xc, Yc;
tf2::Vector3 v;
geometry_msgs::Twist p;

	for(int i=0; i<avoid.size(); i++){		
		Xc_avoid.resize(0); Yc_avoid.resize(0);

		for(int j=0; j<avoid.size(); j++){
			if(i!=j){
				dist=getDistance(avoid[i]->getPose(),avoid[j]->getPose());
				if (dist<d){
					Xc_avoid.push_back(avoid[j]->getPose().position.x);
					Yc_avoid.push_back(avoid[j]->getPose().position.y);
				}
			}
		}
		if(Xc_avoid.size()>0){
			XSep=(accumulate(Xc_avoid.begin(), Xc_avoid.end(), 0.00) /Xc_avoid.size());
			YSep=(accumulate(Yc_avoid.begin(), Yc_avoid.end(), 0.00) /Yc_avoid.size());		
			Xc=XSep-avoid[i]->getPose().position.x;
			Yc=YSep-avoid[i]->getPose().position.y;

			tf2::Matrix3x3 M(avoid[i]->getRotMatrix());

			v.setX(Xc); v.setY(Yc); v.setZ(0);
			
			float vx = (M.tdotx(v));
			float vy = (M.tdoty(v));

			float dtheta=atan2(-vy,-vx);
			if (abs(dtheta)>0.05 && dtheta>0){
				p.angular.x=0.0; p.angular.y=0.0; p.angular.z=0.75*abs(dtheta);				
			}
			else if(abs(dtheta)>0.05 && dtheta<0){
				p.angular.x=0.0; p.angular.y=0.0; p.angular.z=-0.75*abs(dtheta);		
			}
			else{
				p.angular.x=0.0; p.angular.y=0.0; p.angular.z=0.0;				
			}			
			//p.linear.x=-0.2*vx; p.linear.y=-0.2*vy; p.linear.z=0;
		}else{
			p.linear.x=0.0; 	p.linear.y=0.0; 	p.linear.z=0.0;
			p.angular.x=0.0; 	p.angular.y=0.0; 	p.angular.z=0.0;
		}
		avoid[i]->PublishMsg(p);
	}
}



int main(int argc, char** argv)
{
	/* code */
	ros::init(argc, argv, "collision_control");
	ros::NodeHandle nh;

	vector<string> namespaces;
	double d;
	nh.getParam("/namespaces",  namespaces);
	nh.getParam("/avoidRadius", d);

	vector<Avoidance*> avoid;


	for (int i = 0; i < namespaces.size(); i++){
		avoid.push_back(new Avoidance(nh, namespaces[i], namespaces.size()));		
	}

	ros::Rate r(30);

	while (ros::ok()){
		ComputeVel(avoid,d);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <ros/callback_queue.h>
#include <string>
#include <future>

#include <sensor_msgs/Joy.h>

#include <geometry_msgs/Twist.h>

#include <crazyflie_driver/Stop.h>
#include <crazyflie_driver/UpdateParams.h>
#include <crazyflie_driver/Land.h>
#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Position.h>
#include <crazyflie_driver/Hover.h>
#include <crazyflie_driver/GenericLogData.h>


#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <cfswarm/getAverageMapValue.h>

using namespace std;

// Takeoff and landing parameters
double takeoff_height = 0.5; // meters
double takeoff_duration = 2.0; // seconds
double land_duration = 4.0; //seconds
int groupMask = 0;

class cfCtrl{

private:

		//node handle
		ros::NodeHandle nh;

		double x,y,z,yaw;

		//ros::Publisher att_control_pub;
		ros::Publisher vel_control_pub;


		//Velocity messages

		//Subscribers
	    ros::Subscriber joy_sub;
	    ros::Subscriber pose_sub;
		ros::Subscriber fieldvel_sub;
		ros::Subscriber avoidvel_sub;
	    //Services
		ros::ServiceClient stop_client;                            //Stop service client
		ros::ServiceClient takeoff_client;                         //Takeoff service client
		ros::ServiceClient land_client;                            //Land service client
		ros::ServiceClient update_params_client;                   //Update_params service client

		ros::ServiceClient update_brightness_client;               //Update_params service client
		//position topic name
		//std::string positionTopic="position";

		//namespace
		std::string ns;

		//brightness
		double I;

		//Velocity Components
		crazyflie_driver::Hover avoid, field;

		//angular gain and xvel
		double Kw, xvel;
public:

	cfCtrl(){

		ns = ros::this_node::getNamespace();

    	vel_control_pub = nh.advertise<crazyflie_driver::Hover>("cmd_hover", 10, this);

    	//Subsricbers
    	joy_sub =nh.subscribe<sensor_msgs::Joy>("joy",10,&cfCtrl::joy_callback,this);
    	pose_sub = nh.subscribe("cfpose",10,&cfCtrl::pos_callback,this);
    	fieldvel_sub = nh.subscribe("fieldVel", 10, &cfCtrl::getFieldVel, this);
    	avoidvel_sub = nh.subscribe("avoidVel", 10, &cfCtrl::getAvoidVel, this);
    	
    	//Services
    	takeoff_client = nh.serviceClient<crazyflie_driver::Takeoff>("takeoff");
    	land_client = nh.serviceClient<crazyflie_driver::Land>("land");
    	stop_client = nh.serviceClient<crazyflie_driver::Stop>("stop");
    	update_params_client =  nh.serviceClient<crazyflie_driver::UpdateParams>("update_params");

    	update_brightness_client=nh.serviceClient<cfswarm::getAverageMapValue>("/getAverageMapValue");

    	Kw=50;
    	xvel=0.3;
    

	}

	void getFieldVel(const geometry_msgs::Twist& msg){
		crazyflie_driver::Hover vel;
		vel.vx=xvel; vel.vy=0.0; vel.zDistance=0.5;
		//cout<<msg.angular.z<<endl;
		if (msg.angular.z!=0){			//vel.vx=0; 	
			
			vel.yawrate=-msg.angular.z*Kw;
			cout<<"field vel: "<<vel.yawrate<<endl;
		}
		else{			
			vel.yawrate=0.0;		}
		field=vel;
		//vel_control_pub.publish(vel);

	}

	void getAvoidVel(const geometry_msgs::Twist& msg){
		crazyflie_driver::Hover vel;
		 vel.vx=xvel; vel.vy=0.0; vel.zDistance=0.5;
		//cout<<msg.angular.z<<endl;
		if (msg.angular.z!=0){
				
			//vel.vx=0; 			
			vel.yawrate=-msg.angular.z*Kw;
			cout<<"avoid vel: "<<vel.yawrate<<endl;
		}
		else{			
			vel.yawrate=0.0;
		}
		avoid=vel;
		//vel_control_pub.publish(vel);

	}

	void pos_callback(const geometry_msgs::Pose& msg){
		cfswarm::getAverageMapValue brightness_srv;
		brightness_srv.request.posX=int(round(msg.position.y*10));
		brightness_srv.request.posY=int(round(msg.position.x*10));
		update_brightness_client.call(brightness_srv);
		//I=brightness_srv.response.S;
		x=msg.position.x;
		y=msg.position.y;
		z=msg.position.z;
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
	//Will serve as emergency landing and take off for the real experiments
	}	

	void VelocityCmd(){
		if (avoid.yawrate!=0){
			vel_control_pub.publish(avoid);
			cout<<"avoid vel: "<<avoid.yawrate<<endl;
		}else{
			cout<<"field vel: "<<field.yawrate<<endl;
			vel_control_pub.publish(field);
		}


	}

};

int main(int argc, char** argv)
{
	/* code */
	ros::init(argc, argv, "controller");
	cfCtrl crazy;
	ros::Rate r(30);

	while (ros::ok()){
		crazy.VelocityCmd();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
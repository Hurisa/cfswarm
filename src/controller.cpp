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
#include <std_msgs/Float32.h>

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
		ros::Publisher bright_pub;
		std_msgs::Float32 brightness_msg;
		//Velocity messages

		//Subscribers
	    ros::Subscriber joy_sub;
	    ros::Subscriber pose_sub;
		ros::Subscriber fieldvel_sub;
		ros::Subscriber avoidvel_sub;
		ros::Subscriber lightvel_sub;
		ros::Subscriber saturation_sub;
		ros::Subscriber levy_sub;
	    //Services
		ros::ServiceClient stop_client;                            //Stop service client
		ros::ServiceClient takeoff_client;                         //Takeoff service client
		ros::ServiceClient land_client;                            //Land service client
		ros::ServiceClient update_params_client;                   //Update_params service client

		//ros::ServiceClient update_brightness_client;               //Update_params service client
		//position topic name
		//std::string positionTopic="position";

		//namespace
		std::string ns;

		//brightness
		double I;

		//Saturation
		double S;
		bool landcmd;
		//Velocity Components
		crazyflie_driver::Hover avoid, field, light, levy;

		//angular gain and xvel
		double Kw, xvel,height;

		int msg_seq;

		string priority;
public:

	cfCtrl(){

		ns = ros::this_node::getNamespace();

    	vel_control_pub = nh.advertise<crazyflie_driver::Hover>("cmd_hover", 10, this);
    	bright_pub = nh.advertise<std_msgs::Float32>("brightness", 10, this);

    	//Subsricbers
    	joy_sub =nh.subscribe<sensor_msgs::Joy>("joy",10,&cfCtrl::joy_callback,this);
    	pose_sub = nh.subscribe("cfpose",10,&cfCtrl::pos_callback,this);
    	fieldvel_sub = nh.subscribe("fieldVel", 10, &cfCtrl::getFieldVel, this);
    	avoidvel_sub = nh.subscribe("avoidVel", 10, &cfCtrl::getAvoidVel, this);
    	lightvel_sub = nh.subscribe("fireVel", 10, &cfCtrl::getLightVel, this);
    	//saturation_sub = nh.subscribe("saturation", 10, &cfCtrl::getSaturation, this);
    	levy_sub = nh.subscribe("levyVel", 10, &cfCtrl::getLevy, this);
    	
    	//Services
    	takeoff_client = nh.serviceClient<crazyflie_driver::Takeoff>("takeoff");
    	land_client = nh.serviceClient<crazyflie_driver::Land>("land");
    	stop_client = nh.serviceClient<crazyflie_driver::Stop>("stop");
    	update_params_client =  nh.serviceClient<crazyflie_driver::UpdateParams>("update_params");

    	//update_brightness_client=nh.serviceClient<cfswarm::getAverageMapValue>("/getAverageMapValue");

    	
    	
    	nh.getParam("/Kw", Kw);
    	nh.getParam("/xvel", xvel);
    	nh.getParam("/height", height);

    	msg_seq=0;
    	nh.setParam("/landcmd", false);
    	nh.getParam("/priority", priority);

	}

	void getFieldVel(const geometry_msgs::Twist& msg){
		crazyflie_driver::Hover vel;
		vel.header.stamp = ros::Time::now();
		vel.vx=xvel; vel.vy=0.0; vel.zDistance=height;
		//cout<<msg.angular.z<<endl;
		if (msg.angular.z!=0){			//vel.vx=0; 	
			
			vel.yawrate=-msg.angular.z*Kw;
			//cout<<"field vel: "<<vel.yawrate<<endl;
		}
		else{			
			vel.yawrate=0.0;		
		}
		field=vel;
		msg_seq+=1;
		//vel_control_pub.publish(vel);

	}

	void getAvoidVel(const geometry_msgs::Twist& msg){
		crazyflie_driver::Hover vel;
		vel.header.stamp = ros::Time::now();
		 vel.vx=xvel; vel.vy=0.0; vel.zDistance=height;
		//cout<<msg.angular.z<<endl;
		if (msg.angular.z!=0){
				
			//vel.vx=0; 			
			vel.yawrate=-msg.angular.z*Kw;
			//cout<<"avoid vel: "<<vel.yawrate<<endl;
		}
		else{			
			vel.yawrate=0.0;
		}
		avoid=vel;
		msg_seq+=1;
		//vel_control_pub.publish(vel);

	}

	void getLightVel(const geometry_msgs::Twist& msg){
		crazyflie_driver::Hover vel;
		vel.header.stamp = ros::Time::now();
		 vel.vx=xvel; vel.vy=0.0; vel.zDistance=height;
		//cout<<msg.angular.z<<endl;
		if (msg.angular.z!=0){
				
			//vel.vx=0; 			
			vel.yawrate=-msg.angular.z*Kw;
			//cout<<"light vel: "<<vel.yawrate<<endl;
		}
		else{			
			vel.yawrate=0.0;
		}
		light=vel;
		msg_seq+=1;
		//vel_control_pub.publish(vel);

	}

	void getLevy(const geometry_msgs::Twist& msg){
		crazyflie_driver::Hover vel;
		vel.header.stamp = ros::Time::now();
		vel.vx=0.0; vel.vy=0.0; vel.zDistance=height;
		//cout<<msg.angular.z<<endl;
		if (msg.angular.z!=0){
				
			//vel.vx=0; 			
			vel.yawrate=-msg.angular.z*Kw;
			if (vel.yawrate>0){
				vel.yawrate=Kw;
			}
			else if (vel.yawrate<0){
				vel.yawrate=-Kw;
			}
			//cout<<"levy vel: "<<vel.yawrate<<endl;
		}
		else{			
			vel.yawrate=0.0;
		}
		levy=vel;
		msg_seq+=1;


	}

	crazyflie_driver::Hover defaultVel(){
		crazyflie_driver::Hover vel;
		vel.header.stamp = ros::Time::now();
		vel.vx=xvel; vel.vy=0.0; vel.zDistance=height;
		vel.yawrate=0.0;
		msg_seq+=1;
		return vel;

	}

	void pos_callback(const geometry_msgs::Pose& msg){
		x=msg.position.x;
		y=msg.position.y;
		z=msg.position.z;

	}

	// void getSaturation(const std_msgs::Float32& msg){
	// 	S=msg.data;
	// }

	// double Saturation(){
	// 	return S;
	// }

	int getSeq(){
		return msg_seq;
	}


	void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
	//Will serve as emergency landing and take off for the real experiments
	}	

	void VelocityCmd(){
		nh.getParam("/landcmd", landcmd);
		if(!landcmd){
			crazyflie_driver::Hover refvel;
			refvel = defaultVel();
			refvel.header.seq=msg_seq;
	//		vel_control_pub.publish(refvel);

			if (avoid.yawrate!=0){
				refvel=avoid;
				refvel.header.seq=msg_seq;
				vel_control_pub.publish(refvel);
				//cout<<ns<<" "<<"Avoiding "<<avoid.yawrate<<endl;
			}
			else{
				if (field.yawrate!=0){
					refvel=field;
					refvel.header.seq=msg_seq;
					//cout<<"zDist: "<<field.zDistance<<endl;
					//cout<<"field vel: "<<field.yawrate<<endl;
					vel_control_pub.publish(refvel);
					//cout<<ns<<" "<<"Field "<<field.yawrate<<endl;
				}
				else{ 
					if (priority.compare("firefly")){
						if (light.yawrate!=0){		
							refvel=light;	
							refvel.header.seq=msg_seq;
							//cout<<light.yawrate<<endl;
							vel_control_pub.publish(refvel);
							//cout<<ns<<" "<<"light"<<endl;				
						}
						else{
							if (levy.yawrate!=0){		
							refvel=levy;
							refvel.header.seq=msg_seq;	
							vel_control_pub.publish(refvel);				
							}
							else{
							refvel = defaultVel();
							refvel.header.seq=msg_seq;
							vel_control_pub.publish(refvel);
							//cout<<ns<<" "<<"defaultVel"<<endl;
							}
						}
					}
					else{
						if (levy.yawrate!=0){		
							refvel=levy;
							refvel.header.seq=msg_seq;	
							vel_control_pub.publish(refvel);				
						}
						else{
							if (light.yawrate!=0){		
							refvel=light;	
							refvel.header.seq=msg_seq;
							//cout<<light.yawrate<<endl;
							vel_control_pub.publish(refvel);
							//cout<<ns<<" "<<"light"<<endl;				
							}
							else{
							refvel = defaultVel();
							refvel.header.seq=msg_seq;
							vel_control_pub.publish(refvel);
							//cout<<ns<<" "<<"defaultVel"<<endl;
							}
						}

					}				
				}
			}
		}
		else{
			land();
		}
	}

	void land(){
		double sleep(5.0);
		cout<<"Sending landing command"<<endl;		
		crazyflie_driver::Land land_srv;
        land_srv.request.height = 0.1;
        land_srv.request.duration = ros::Duration(sleep);
        land_srv.request.groupMask = 0;
        land_client.call(land_srv);
        ros::Duration(sleep).sleep();
        // string kill;
        // kill="rosnode kill "+ns+"/controller";
        // system("rosnode kill "+ns+"/controller");
		
	}
};

int main(int argc, char** argv)
{
	/* code */
	ros::init(argc, argv, "controller");
	cfCtrl crazy;
	ros::Rate r(100);
	
	while (ros::ok()){

	
		crazy.VelocityCmd();

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
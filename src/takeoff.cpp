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
double takeoff_height = 0.2; // meters
double takeoff_duration = 2.0; // seconds
double land_duration = 2.0; //seconds
int groupMask = 0;

class cfCtrl{

private:

		//node handle
		ros::NodeHandle nh;
        //Stop service client
		ros::ServiceClient takeoff_client;                         //Takeoff service client
		ros::ServiceClient update_params_client;
	
public:

	cfCtrl(){
    	//Services
    	takeoff_client = nh.serviceClient<crazyflie_driver::Takeoff>("takeoff");    	
    	update_params_client =  nh.serviceClient<crazyflie_driver::UpdateParams>("update_params");  	
	}


	void takeoff(){

		crazyflie_driver::UpdateParams m_params;
    	m_params.request.params.push_back("commander/enHighLevel");
    	ros::param::set("commander/enHighLevel" , 1);
    	update_params_client.call(m_params);
		
		crazyflie_driver::Takeoff takeoff_srv;
        takeoff_srv.request.height = takeoff_height;
        takeoff_srv.request.duration = ros::Duration(takeoff_duration);
        takeoff_srv.request.groupMask = groupMask;
        takeoff_client.call(takeoff_srv);
		ros::Duration(0.5).sleep();
	}

};

int main(int argc, char** argv)
{
	/* code */
	ros::init(argc, argv, "takeoff");
	cfCtrl crazy;
	ros::Rate r(30);
	crazy.takeoff();
	// while (ros::ok()){

	// 	ros::spinOnce();
	//  	r.sleep();
	// }
	//return 0;
}
#include "ros/ros.h"
#include "fyt_mae/fyt_commons.h"
#include "cmg_msgs/Signal.h"
#include "cmg_msgs/DynamixelStateList.h"
#include "sensor_msgs/Imu.h"

#define MAX_TEMP 60
#define MAX_VIT 3.14

class Controller {
	public:
		Controller(): n("~") {
			pub_alert = n.advertise<cmg_msgs::Signal>("/mae/signal", 1);
			sub_dxl = n.subscribe("/gimbal/state", 10, &Controller::dxl_cb, this);
			sub_imu = n.subscribe("/imu/filtre", 10, &Controller::imu_cb, this);
		}
	private:
		ros::NodeHandle n;
		ros::Publisher pub_alert;
		ros::Subscriber sub_dxl, sub_imu;

		void alert(int sig) {
			cmg_msgs::Signal s;
			s.signal = sig;
			pub_alert.publish(s);
		}

		void dxl_cb(const cmg_msgs::DynamixelStateList::ConstPtr & msg) {
			for (int i = 0; i < msg->states.size(); i++) {
				if (msg->states[i].present_temperature > MAX_TEMP) {
					ROS_WARN("Dynamixel #%d is hot: %dC", i, msg->states[i].present_temperature);
					alert(SIG_HOT);
					return;
				}
			} 
		}

		void imu_cb(const sensor_msgs::Imu::ConstPtr & msg) {
			auto vel = msg->angular_velocity;
			double vit = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z; 
			if (vit > MAX_VIT) {
					ROS_WARN("Angular velocity is high: %f", vit);
					alert(SIG_FAST);	
			} 
		}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "safety");
	Controller c;
	ros::spin();
}

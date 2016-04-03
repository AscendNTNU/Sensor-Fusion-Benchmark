#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <Eigen/Dense>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub_ = n_.advertise<geometry_msgs::PoseStamped>("/optitrack_tf/pose", 100);
		//Topic you want to subscribe
		sub_ = n_.subscribe("vrpn_client_node/Trackable_1/pose", 100, &SubscribeAndPublish::callback, this);
	}
	void callback(const geometry_msgs::PoseStamped& input)
	{
		pose_ = input;
		pose_.pose.position.z = input.pose.position.y;
		pose_.pose.position.y = -input.pose.position.z;
		pose_.pose.orientation.y = -input.pose.orientation.z;
		pose_.pose.orientation.z = input.pose.orientation.y;
		pose_.header = input.header;
		pose_.header.frame_id = "fcu";
		pub_.publish(pose_);

		tf2::fromMsg(pose_.pose.orientation, tmp_pose_);
		tf2::Matrix3x3(tmp_pose_).getRPY(roll_, pitch_, yaw_);
       		q_.setRPY(roll_, pitch_, yaw_);
	        static tf::TransformBroadcaster br_;
       		transform_.setOrigin( tf::Vector3(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z) );
        	transform_.setRotation(q_);
        	br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "fcu", "body"));
	}
private:
	tf2::Quaternion tmp_pose_;
	tf::Quaternion q_;
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Subscriber sub_;
	geometry_msgs::PoseStamped pose_;
	tf2::Quaternion orientation;
	tf2::Quaternion tmp;

	double roll_, pitch_, yaw_;

	tf::Transform transform_;
};//End of class SubscribeAndPublish
int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "subscribe_and_publish");
	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}
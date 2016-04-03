#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub_ = n_.advertise<geometry_msgs::PoseStamped>("/benchmark/pose", 100);
		//Topic you want to subscribe
		sub_kf_ = n_.subscribe("kalman_filter/pose", 100, &SubscribeAndPublish::callbackKF, this);
		sub_ot_ = n_.subscribe("optitrack_tf/pose", 100, &SubscribeAndPublish::callbackOT, this);
	}
	void callbackKF(const geometry_msgs::PoseStamped& input)
	{
		pose_kf_ = input;
		pose_diff_.pose.position.x = pose_kf_.pose.position.x - pose_ot_.pose.position.x;
		pose_diff_.pose.position.y = pose_kf_.pose.position.y - pose_ot_.pose.position.y;
		pose_diff_.pose.position.z = pose_kf_.pose.position.z - pose_ot_.pose.position.z;
		
		pose_diff_.header = input.header;
		pose_diff_.header.frame_id = "fcu";
		
		tf2::fromMsg(pose_.pose.orientation, tmp_pose_);
		tf2::Matrix3x3(tmp_pose_).getRPY(roll_kf_, pitch_kf_, yaw_kf_);
		roll_diff_ = roll_kf_ - roll_ot_;
		pitch_diff_ = pitch_kf_ - pitch_ot_;
		yaw_diff_ = yaw_kf_ - yaw_ot_;
       		q_.setRPY(roll_diff_, pitch_diff_, yaw_diff_);
       		pose_diff_.pose.orientation.w = q_.w();
       		pose_diff_.pose.orientation.x = q_.x();
       		pose_diff_.pose.orientation.y = q_.y();
       		pose_diff_.pose.orientation.z = q_.z();
       		pub_.publish(pose_diff_);
	        static tf::TransformBroadcaster br_;
       		transform_.setOrigin( tf::Vector3(pose_diff_.pose.position.x, pose_diff_.pose.position.y, pose_diff_.pose.position.z) );
        	transform_.setRotation(q_);
        	br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "fcu", "body"));
	}
	void callbackOT(const geometry_msgs::PoseStamped& input)
	{
		pose_ot_ = input;
		tf2::fromMsg(pose_ot_.pose.orientation, tmp_pose_);
		tf2::Matrix3x3(tmp_pose_).getRPY(roll_ot_, pitch_ot_, yaw_ot_);
	}
	
private:
	tf2::Quaternion tmp_pose_;
	tf::Quaternion q_;
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Subscriber sub_;
	geometry_msgs::PoseStamped pose_kf_;
	geometry_msgs::PoseStamped pose_ot_;	
	geometry_msgs::PoseStamped pose_diff_;
	tf2::Quaternion orientation;
	tf2::Quaternion tmp;

	double roll_kf_, pitch_kf_, yaw_kf_, roll_ot_, pitch_ot_, yaw_ot_, roll_diff_, pitch_diff_, yaw_diff_;

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

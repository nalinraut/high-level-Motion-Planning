
// desc: listens for tf & publishes as topics
//        - the original purpose was to have the same information at a guaranteed constant rate
//        
// author: ks
// date: (old)
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

nav_msgs::Odometry global_odom;
nav_msgs::Odometry odom_to_base_link;

int main(int argc, char** argv) {

	ros::init(argc,argv,"global_odom_publisher");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("global_odom_pose", 10);
	ros::Publisher pub_odom_to_base_link = n.advertise<nav_msgs::Odometry>("tf_odom_to_base_link", 10);

	ros::Rate r( 30 );

	tf::StampedTransform transform;
	tf::TransformListener listener;
	
	while ( n.ok() ) {
	  try {
			tf::StampedTransform transform;
			tf::StampedTransform odom_transform;

      // map -> base_link 
			listener.lookupTransform("map", "base_link", ros::Time(0), transform);
		
			tf::pointTFToMsg(tf::Point(transform.getOrigin()), global_odom.pose.pose.position);
			tf::Quaternion tf_quat = 	transform.getRotation();
			tf::quaternionTFToMsg(tf_quat, global_odom.pose.pose.orientation );

			global_odom.header.stamp = transform.stamp_;
			global_odom.header.frame_id = "map";
			global_odom.child_frame_id = "base_link";

			pub.publish(global_odom);

      // @TODO just listen to "odom" msg
      // odom -> base_link
			listener.lookupTransform("odom", "base_link", ros::Time(0), odom_transform);

			tf::pointTFToMsg(tf::Point(odom_transform.getOrigin()), odom_to_base_link.pose.pose.position);
			tf_quat = 	odom_transform.getRotation();
			tf::quaternionTFToMsg(tf_quat, odom_to_base_link.pose.pose.orientation );

			odom_to_base_link.header.stamp = odom_transform.stamp_;
		  odom_to_base_link.header.frame_id = "odom";
			odom_to_base_link.child_frame_id = "base_link";

			pub_odom_to_base_link.publish(odom_to_base_link);
		} catch (tf::TransformException ex) {}

		r.sleep();
	}	
}

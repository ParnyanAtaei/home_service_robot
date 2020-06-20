#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double x_goal_1 = 6;
double y_goal_1 = 3;
double x_goal_2 = 0;
double y_goal_2 = 7;


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {

   marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

   odom_sub_ = n_.subscribe("/odom", 10, &SubscribeAndPublish::position_callback, this);

   reached_goal_1 = false;
   reached_goal_2 = false;
  }

  void position_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void draw_marker(double x, double y);

private:
  ros::NodeHandle n_; 
  ros::Publisher marker_pub_;
  ros::Subscriber odom_sub_;
  bool reached_goal_1;
  bool reached_goal_2;

};


void SubscribeAndPublish::draw_marker(double x, double y)
   {

     uint32_t shape = visualization_msgs::Marker::CUBE;
     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     marker.header.frame_id = "map";
     marker.header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "basic_shapes";
     marker.id = 0;
 
     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     marker.type = visualization_msgs::Marker::CUBE;

     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::MODIFY;

 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     marker.pose.position.x = x;
     marker.pose.position.y = y;
     marker.pose.position.z = 0;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
 
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = 0.3;
     marker.scale.y = 0.3;
     marker.scale.z = 0.3;
   
     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;

     marker.lifetime = ros::Duration(1); 
     marker_pub_.publish( marker );

 }

void SubscribeAndPublish::position_callback(const nav_msgs::Odometry::ConstPtr& msg){

 double x = msg->pose.pose.position.x; 
 double y = msg->pose.pose.position.y;

 if(fabs(x - x_goal_1) <  0.1 && fabs(y - y_goal_1) <  0.1){
    reached_goal_1 = true;
 }


 if(reached_goal_1 && fabs(x - x_goal_2) <  0.1 && fabs(y - y_goal_2) <  0.1 ){
    reached_goal_2 = true;
 }

  //ROS_INFO("%d, %d ",reached_goal_1, reached_goal_2);
  if(reached_goal_1 && reached_goal_2){
    draw_marker(x_goal_2, y_goal_2);
  }
  else if(!reached_goal_2 && !reached_goal_1){
    draw_marker(x_goal_1, y_goal_1);
  }

 
}


int main( int argc, char** argv )
 {
   ros::init(argc, argv, "add_markers");

   SubscribeAndPublish SAPObject;

   ros::spin();

   //ros::Rate r(1);

         return 0;

}


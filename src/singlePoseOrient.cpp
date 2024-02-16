// Import libreries
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

// The srv class for the service.
#include "chesslab_setup/getpiecepose.h"

// Global variables
tf2_ros::Buffer tfBuffer;


bool getPiecePose(chesslab_setup::getpiecepose::Request  &req, chesslab_setup::getpiecepose::Response &res)
{
    // Callback function of the advertised service. 
    //It requests 2 strings and returns one tf that relates the 2 inputs.
    
    // Local variables
    static geometry_msgs::TransformStamped transformStamped;
    //ROS_INFO("The pose of piece %s is:", req.id1.c_str());

    // Get the pose of id1 with respect to id2 
    for(int i=0; i<3; i++){
        try{
          transformStamped = tfBuffer.lookupTransform(req.id1, req.id2, ros::Time(0));//, ros::Duration(0.1));
        }
        catch (tf2::TransformException ex ){
            ROS_ERROR("%d - %s",i,ex.what());
            // Error
            continue;
        }
    }
    
    // Return the pose
    res.stampedpose.transform.translation.x = transformStamped.transform.translation.x;
    res.stampedpose.transform.translation.y = transformStamped.transform.translation.y;
    res.stampedpose.transform.translation.z = -transformStamped.transform.translation.z;
    res.stampedpose.transform.rotation.x = transformStamped.transform.rotation.x;
    res.stampedpose.transform.rotation.y = transformStamped.transform.rotation.y;
    res.stampedpose.transform.rotation.z = transformStamped.transform.rotation.z;
    res.stampedpose.transform.rotation.w = transformStamped.transform.rotation.w;

    // Print the pose in the terminal
    /*ROS_INFO("x: %f", res.stampedpose.transform.translation.x);
    ROS_INFO("y: %f", res.stampedpose.transform.translation.y);
    ROS_INFO("z: %f", res.stampedpose.transform.translation.z);
    ROS_INFO("qx: %f", res.stampedpose.transform.rotation.x);
    ROS_INFO("qy: %f", res.stampedpose.transform.rotation.y);
    ROS_INFO("qz: %f", res.stampedpose.transform.rotation.z);
    ROS_INFO("qw: %f", res.stampedpose.transform.rotation.w);*/

    return true;
}


int main(int argc, char **argv){
    // Node that contains a server of a service. It returns a tf between 2 given id strings. The service is called "getpiecepose".
    
    // Initialization of the node
    ros::init(argc, argv, "singlePoseOrient");
    ros::NodeHandle nh;

    // The node provides a service to get the pose of a given piece identified by its ID (string) wrt to another id
    ros::ServiceServer serviceGetPiecePose = nh.advertiseService("chesslab_setup/getpiecepose", getPiecePose);

    tf2_ros::TransformListener tfListener(tfBuffer);
    ROS_INFO("ready!");
    ros::Rate rate(2);
    
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}

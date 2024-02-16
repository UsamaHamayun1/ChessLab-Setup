#include <iostream>
#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

//#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
#include <chesslab_setup/ik.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_ik");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to thest the ik service****");    
    
    //call the inversekin service
    ros::service::waitForService("/chesslab_setup/inversekin");
    ros::ServiceClient inversekin_client = node.serviceClient<chesslab_setup::ik>("/chesslab_setup/inversekin");
    chesslab_setup::ik inversekin_srv;
       
    inversekin_srv.request.pose.position.x = 0.422;
    inversekin_srv.request.pose.position.y = -0.039;
    inversekin_srv.request.pose.position.z = 0.100;
    inversekin_srv.request.pose.orientation.x = 0.707;
    inversekin_srv.request.pose.orientation.y = -0.707;
    inversekin_srv.request.pose.orientation.z = 0.0;
    inversekin_srv.request.pose.orientation.w = 0.0;
    /*
    inversekin_srv.request.pose.position.x = 0;
    inversekin_srv.request.pose.position.y = 0;
    inversekin_srv.request.pose.position.z = 10;
    inversekin_srv.request.pose.orientation.x = 0.707;
    inversekin_srv.request.pose.orientation.y = -0.707;
    inversekin_srv.request.pose.orientation.z = 0.0;
    inversekin_srv.request.pose.orientation.w = 0.0;
    */
    inversekin_client.call(inversekin_srv);
    
    ROS_INFO_STREAM("Robot Pose: [" << 
        inversekin_srv.request.pose.position.x << ", " <<
        inversekin_srv.request.pose.position.y << ", " <<
        inversekin_srv.request.pose.position.z << ", " << 
        inversekin_srv.request.pose.orientation.x << ", " << 
        inversekin_srv.request.pose.orientation.y << ", " <<
        inversekin_srv.request.pose.orientation.z << ", " <<
        inversekin_srv.request.pose.orientation.w << "]");
    
    std::stringstream sstr;
    if(inversekin_srv.response.status)
    {
        sstr<<"The computed ik is:"<<std::endl;
        for(int i=0; i<inversekin_srv.response.ik_solution.size(); i++)
        {
            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << inversekin_srv.response.ik_solution[i].ik[j] <<", ";            
            }
            sstr << inversekin_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
    }
    else{
        ROS_INFO("Not able to compute the ik");
    }
}

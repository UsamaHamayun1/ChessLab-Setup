#include <iostream>
#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>
#include <std_srvs/Empty.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_collision");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the motion of the robot and the collision-check ****");

   //Set the initial scene
    if(argc!=1 && strcmp(argv[1], "initialize") == 0){
        std::cout<<"Initializing SCENE"<<std::endl;
        ros::service::waitForService("/chesslab_setup/resetscene");
        ros::ServiceClient resetscene_client = node.serviceClient<std_srvs::Empty>("/chesslab_setup/resetscene");
        std_srvs::Empty resetscene_srv;
        resetscene_client.call(resetscene_srv);
    }
    else{
        std::cout<<"Skipping Initialization"<<std::endl;
    }

    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
    std::cin.get();

    //Move robots
    ros::service::waitForService("/chesslab_setup/setrobconf");
    ros::ServiceClient setrobconf_client = node.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");
    chesslab_setup::setrobconf setrobconf_srv;

    //collision robA with kingB:
    //rosservice call /chesslab_setup/setrobconf "conf: [1.02,-1.181,1.602,1.420,1.495,1.0,0.5]"
    setrobconf_srv.request.conf.resize(7);
    setrobconf_srv.request.conf[0]  =  1.02;
    setrobconf_srv.request.conf[1]  = -1.181;
    setrobconf_srv.request.conf[2]  =  1.602;
    setrobconf_srv.request.conf[3]  =  1.420;
    setrobconf_srv.request.conf[4]  =  1.495;
    setrobconf_srv.request.conf[5]  =  1.0;
    setrobconf_srv.request.conf[6]  =  0.5;

    std::cout << "Moving robots to: [" <<
        setrobconf_srv.request.conf[0] << ", " <<
        setrobconf_srv.request.conf[1] << ", " <<
        setrobconf_srv.request.conf[2] << ", " <<
        setrobconf_srv.request.conf[3] << ", " <<
        setrobconf_srv.request.conf[4] << ", " <<
        setrobconf_srv.request.conf[5] << ", " <<
        setrobconf_srv.request.conf[6] << "]" << std::endl;

    setrobconf_client.call(setrobconf_srv);

    if(setrobconf_srv.response.incollision == true)
    {
        std::cout << "The configuration is not collision-free"<< std::endl;
        std::cout << setrobconf_srv.response.msg;
        std::cout << "The collided obstacle aruco ID is " << setrobconf_srv.response.obj << std::endl;
    }
    else{
        std::cout << setrobconf_srv.response.msg;
    }

}

#include <iostream>
#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
#include <chesslab_setup/attachobs2robot.h>
#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>
#include <std_srvs/Empty.h>



int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_dettach");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the attach/dettach services ****");

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

    ROS_INFO("**** Move Robot and attach piece ****");

    ros::service::waitForService("/chesslab_setup/setrobconf");
    ros::ServiceClient setrobconf_client = node.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");
    chesslab_setup::setrobconf setrobconf_srv;

    //rosservice call /chesslab_setup/setrobconf "conf: [1.206,-1.326,1.734,1.156,1.615,1.112, 0.498291398]"
    //Sample: 0.692 0.289 0.776 0.684 0.757 0.677 0.571
    setrobconf_srv.request.conf.resize(7);
    setrobconf_srv.request.conf[0]  =  1.206;
    setrobconf_srv.request.conf[1]  = -1.326;
    setrobconf_srv.request.conf[2]  =  1.734;
    setrobconf_srv.request.conf[3]  =  1.156;
    setrobconf_srv.request.conf[4]  =  1.615;
    setrobconf_srv.request.conf[5]  =  1.112;
    setrobconf_srv.request.conf[6]  =  0.498291398;

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

    //call the attach service
    ros::service::waitForService("/chesslab_setup/attachobs2robot");
    ros::ServiceClient attachobs2robot_client = node.serviceClient<chesslab_setup::attachobs2robot>("/chesslab_setup/attachobs2robot");
    chesslab_setup::attachobs2robot attachobs2robot_srv;

    attachobs2robot_srv.request.robotName =  "team_A";
    attachobs2robot_srv.request.objarucoid =  216;
    //The following delay is necessary to let the robot reach the final configuration before calling the attach (the exact value not analyzed)
    ros::Duration(2).sleep();
    attachobs2robot_client.call(attachobs2robot_srv);

    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO CONTINUE..."<<std::endl;
    std::cin.get();

    ROS_INFO("**** Move Robot with attached piece, detacches it and moves without it****");

    //rosservice call /chesslab_setup/setrobconf "conf: [1.169,-1.28,1.35,1.25,1.58,1.09, 0.498291398]"
    setrobconf_srv.request.conf.resize(7);
    setrobconf_srv.request.conf[0]  =  1.169;
    setrobconf_srv.request.conf[1]  = -0.798;
    setrobconf_srv.request.conf[2]  =  0.898;
    setrobconf_srv.request.conf[3]  =  1.495;
    setrobconf_srv.request.conf[4]  =  1.615;
    setrobconf_srv.request.conf[5]  =  1.112;
    setrobconf_srv.request.conf[6]  =  0.498291398;


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


    //call the dettach service
    ros::service::waitForService("/chesslab_setup/dettachobs");
    ros::ServiceClient dettachobs_client = node.serviceClient<chesslab_setup::dettachobs>("/chesslab_setup/dettachobs");
    chesslab_setup::dettachobs dettachobs_srv;

    dettachobs_srv.request.robotName =  "team_A";
    dettachobs_srv.request.objarucoid =  216;
    //The following delay is necessary to let the robot reach the final configuration before calling the attach (the exact value not analyzed)
    ros::Duration(2).sleep();
    dettachobs_client.call(dettachobs_srv);

    //move again
    //rosservice call /chesslab_setup/setrobconf "conf: [1.206,-1.326,1.734,1.156,1.615,1.112, 0.498291398]"
    //Sample: 0.692 0.289 0.776 0.684 0.757 0.677 0.573
    setrobconf_srv.request.conf.resize(7);
    setrobconf_srv.request.conf[0]  =  1.206;
    setrobconf_srv.request.conf[1]  = -1.326;
    setrobconf_srv.request.conf[2]  =  1.734;
    setrobconf_srv.request.conf[3]  =  1.156;
    setrobconf_srv.request.conf[4]  =  1.615;
    setrobconf_srv.request.conf[5]  =  1.112;
    setrobconf_srv.request.conf[6]  =  0.498291398;

    std::cout << "Moving robots to: [" <<
        setrobconf_srv.request.conf[0] << ", " <<
        setrobconf_srv.request.conf[1] << ", " <<
        setrobconf_srv.request.conf[2] << ", " <<
        setrobconf_srv.request.conf[3] << ", " <<
        setrobconf_srv.request.conf[4] << ", " <<
        setrobconf_srv.request.conf[5] << ", " <<
        setrobconf_srv.request.conf[6] << "]" << std::endl;

    setrobconf_client.call(setrobconf_srv);
}

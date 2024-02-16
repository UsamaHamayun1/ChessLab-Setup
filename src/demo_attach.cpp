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
//#include <kautham/VisualizeScene.h>



int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_attach");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the attach of a piece and the collision-check ****");

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
        std::cout << "The configuration is not collision-free"<<std::endl;
        std::cout << setrobconf_srv.response.msg;
        std::cout << "The collided obstacle aruco ID is " << setrobconf_srv.response.obj << std::endl;
        /*
        std::cout << "Waiting for service VisualizeScene" << std::endl;
        ros::service::waitForService("kautham_node/VisualizeScene");
        std::cout << "Service VisualizeScene available" << std::endl;
        ros::ServiceClient visualizescene_client = node.serviceClient<kautham::VisualizeScene>("kautham_node/VisualizeScene");
        kautham::VisualizeScene visualizescene_srv;
        std::cout << "Calling VisualizeScene" << std::endl;
        visualizescene_client.call(visualizescene_srv);
        */
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
    std::cout<<"\n PRESS A KEY TO CONTINUE..."<<std::endl;
    std::cin.get();

    ROS_INFO("**** Move Robot with attached piece (and checks for collisions) ****");

    //rosservice call /chesslab_setup/setrobconf "conf: [1.206,-1.326,1.35,1.156,1.615,1.112, 0.498291398]"
    //Sample: 0.692 0.289 0.776 0.684 0.757 0.677 0.571
    setrobconf_srv.request.conf.resize(7);
    setrobconf_srv.request.conf[0]  =  1.206;
    setrobconf_srv.request.conf[1]  = -1.326;
    setrobconf_srv.request.conf[2]  =  1.35;
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

}

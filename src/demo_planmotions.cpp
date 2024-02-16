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
#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_planmotions");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the planning of motions using the FF task planner ****");
    
    //call the planmovement service
    ros::service::waitForService("/chesslab_setup/planmovement");
    ros::ServiceClient planmovement_client = node.serviceClient<chesslab_setup::ffplan>("/chesslab_setup/planmovement");
    chesslab_setup::ffplan planmovement_srv;
       
    /* MOVE ACTION */
    std::cout<<"\n*** MOVE ACTION ***"<<std::endl;
    //Initial state
    //BLACK_PAWN_1 in A4
    planmovement_srv.request.init.objarucoid.push_back(201);
    planmovement_srv.request.init.occupiedcells.push_back("A4");
    //Free cell B5
    planmovement_srv.request.init.freecells.push_back("B5");
    //Goal state
    //BLACK_PAWN_1 in B5
    planmovement_srv.request.goal.objarucoid.push_back(201);
    planmovement_srv.request.goal.occupiedcells.push_back("B5");
    //Free cell A4
    planmovement_srv.request.goal.freecells.push_back("A4");

    //Solve
    planmovement_client.call(planmovement_srv);
    
    std::cout<<"Initial state:"<<std::endl;
    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
    {
        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
    }
    std::cout<<"Goal state:"<<std::endl;
    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
    {
        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
    }
    std::cout<<"Plan:"<<std::endl;
    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
    {
        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
    }

    //Clear the query
    planmovement_srv.request.init.objarucoid.clear();
    planmovement_srv.request.init.occupiedcells.clear();
    planmovement_srv.request.init.freecells.clear();
    planmovement_srv.request.goal.objarucoid.clear();
    planmovement_srv.request.goal.occupiedcells.clear();
    planmovement_srv.request.goal.freecells.clear();
    
    /* KILL ACTION */
    std::cout<<"\n*** KILL ACTION ***"<<std::endl;
    //Initial state
    //BLACK_PAWN_1 in A4
    planmovement_srv.request.init.objarucoid.push_back(201);
    planmovement_srv.request.init.occupiedcells.push_back("A4");
    //WHITE_PAWN_1 in B5
    planmovement_srv.request.init.objarucoid.push_back(301);
    planmovement_srv.request.init.occupiedcells.push_back("B5");
    //Goal state
    //BLACK_PAWN_1 in B5
    planmovement_srv.request.goal.objarucoid.push_back(201);
    planmovement_srv.request.goal.occupiedcells.push_back("B5");
    //WHITE_PAWN_1 KILLED
    planmovement_srv.request.killed = 301;
    //Free cell A4
    planmovement_srv.request.goal.freecells.push_back("A4");

    //Solve
    planmovement_client.call(planmovement_srv);

    std::cout<<"Initial state:"<<std::endl;
    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
    {
        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
    }
    std::cout<<"Goal state:"<<std::endl;
    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
    {
        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
    }
    std::cout<<"Plan:"<<std::endl;
    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
    {
        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
    }
    
    //Clear the query
    planmovement_srv.request.init.objarucoid.clear();
    planmovement_srv.request.init.occupiedcells.clear();
    planmovement_srv.request.init.freecells.clear();
    planmovement_srv.request.goal.objarucoid.clear();
    planmovement_srv.request.goal.occupiedcells.clear();
    planmovement_srv.request.goal.freecells.clear();
    planmovement_srv.request.killed=0;
    
    /* CASTLE ACTION */
    std::cout<<"\n*** CASTLE ACTION ***"<<std::endl;
    //Initial state
    //WHITE_TOWER_1 in A1
    planmovement_srv.request.init.objarucoid.push_back(309);
    planmovement_srv.request.init.occupiedcells.push_back("A1");
    //WHITE_KING in E1
    planmovement_srv.request.init.objarucoid.push_back(316);
    planmovement_srv.request.init.occupiedcells.push_back("E1");
    //Free cell D1 and C1
    planmovement_srv.request.init.freecells.push_back("D1");
    planmovement_srv.request.init.freecells.push_back("C1");
    //Goal state
    //WHITE_TOWER_1 in D1
    planmovement_srv.request.goal.objarucoid.push_back(309);
    planmovement_srv.request.goal.occupiedcells.push_back("D1");
    //WHITE_KING in C1
    planmovement_srv.request.goal.objarucoid.push_back(316);
    planmovement_srv.request.goal.occupiedcells.push_back("C1");
    //Free cell A4
    planmovement_srv.request.goal.freecells.push_back("A1");
    planmovement_srv.request.goal.freecells.push_back("E1");

    //Solve
    planmovement_client.call(planmovement_srv);

    std::cout<<"Initial state:"<<std::endl;
    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
    {
        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
    }
    std::cout<<"Goal state:"<<std::endl;
    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
    {
        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
    }
    std::cout<<"Plan:"<<std::endl;
    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
    {
        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
    }
    
    
}

// Include libreies and services files
#include <ros/ros.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <chesslab_setup/set_point_to_point_trajectory.h>
#include <chesslab_setup/set_goal_tolerances.h>
#include <chesslab_setup/move_robot_trajectory.h>
#include <chesslab_setup/move_robot_trajectory_safe.h>
#include <geometry_msgs/Pose.h>
#include <chesslab_setup/setrobconf.h>
#include <chesslab_setup/setobjpose.h>
#include <chesslab_setup/ik.h>
#include <chesslab_setup/attachobs2robot.h>
#include <chesslab_setup/dettachobs.h>
#include <std_srvs/Empty.h>
#include <chesslab_setup/pickAndPlace.h>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <chesslab_setup/setConf.h>
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

// Define the class
class move_robot_class{
    // Private objects
    private:
        // inicialization of variables
        ros::NodeHandle nh;
        std::string nodename;
        chesslab_setup::ik inversekin_srv;
        ros::ServiceServer moveRobot;
        geometry_msgs::Pose initialPose;
        geometry_msgs::Pose finalPose;
        ros::ServiceClient inversekin_client;

        ///////////////////////////////////////////
        /*
        //Create the request and response objects for setting the traj
        chesslab_setup::set_point_to_point_trajectory settraj_srv;
        chesslab_setup::set_point_to_point_trajectory::Request reqsetraj;
        chesslab_setup::set_point_to_point_trajectory::Response respsettraj;
        Create the request and response objects for moving the robot
        chesslab_setup::move_robot_trajectory moverobottraj_srv;
        chesslab_setup::move_robot_trajectory::Request reqmovetraj;
        chesslab_setup::move_robot_trajectory::Response respmovetraj;
        Create the request and response objects for moving the robot with limits
        chesslab_setup::move_robot_trajectory_safe::Request reqmovetrajsafe;
        chesslab_setup::move_robot_trajectory_safe::Response respmovetrajsafe;
        */

        ros::ServiceClient client_settraj, newConf_client;
        //ros::ServiceClient client_setgoaltol;
	    ros::ServiceClient client_moverobottraj;
	    //ros::ServiceClient client_moverobottrajsafe;

        double moveduration;

    // Public objects
    public:
    // This node advertises a service called pick and place. It uses the information recibed to compute the inverse kinematics and move the robot.
        move_robot_class(std::string name){
            nodename = name;
            nh = ros::NodeHandle(nodename);

            // pickAndPlace service server
            moveRobot = nh.advertiseService("pickAndPlace",&move_robot_class::pickAndPlace, this);
            // ik service client
            inversekin_client = nh.serviceClient<chesslab_setup::ik>("/chesslab_setup/inversekin");
            // setConf service
            ros::service::waitForService("/setConf");
            newConf_client = nh.serviceClient<chesslab_setup::setConf>("/setConf");
            //ros::service::waitForService("/set_point_to_point_trajectory");
            //client_settraj = nh.serviceClient<chesslab_setup::set_point_to_point_trajectory>("set_point_to_point_trajectory");
            //client_setgoaltol = nh.serviceClient<chesslab_setup::set_goal_tolerances>("set_goal_tolerances");
            client_moverobottraj = nh.serviceClient<chesslab_setup::move_robot_trajectory>("move_robot_trajectory");
            //client_moverobottrajsafe = nh.serviceClient<chesslab_setup::move_robot_trajectory_safe>("move_robot_trajectory_safe");

            moveduration = 5.0;
        }

        // Pick and place callback
        bool pickAndPlace(chesslab_setup::pickAndPlace::Request &req,
                         chesslab_setup::pickAndPlace::Response &resp){

            // Save the initial position of the pick and place
            initialPose.position.x = req.poseInitial.position.x;
            initialPose.position.y = req.poseInitial.position.y;
            initialPose.position.z = req.poseInitial.position.z;
            initialPose.orientation.x = -0.707;
            initialPose.orientation.y = 0.707;
            initialPose.orientation.z = 0;
            initialPose.orientation.w = 0;

            // Check the correctness of the orientation
            if(req.orientationInitial){
                initialPose.orientation.x = 0.707;
                initialPose.orientation.y = -0.707;
            }

            // Save the final position of the pick and place
            finalPose.position.x = -req.poseFinal.position.x;
            finalPose.position.y = -req.poseFinal.position.y;
            finalPose.position.z = req.poseFinal.position.z;
            finalPose.orientation.x = -0.707;
            finalPose.orientation.y = 0.707;
            finalPose.orientation.z = 0;
            finalPose.orientation.w = 0;

            // Check the correctness of the orientation
            if(req.orientationFinal){
                finalPose.orientation.x = 0.707;
                finalPose.orientation.y = -0.707;
            }

            // Define the number of instances the linear movement will be split in
            double robotPositionHeight = 0.20;
            int numPoints = 10;
            double saveMargin = 0.002;

            // For each step of the movement, the ik is computed and selected acording to the requirements
            ros::service::waitForService("/chesslab_setup/inversekin");
            std::vector<std::vector<double>> allIK; //Contains all the configurations of the trajectory
            std::vector<std::vector<double>> oneIK;
            std::vector<double> ik_selected;
            for(int i = 0; i<numPoints; ++i){
                oneIK = getIK(initialPose, numPoints, robotPositionHeight, saveMargin, i);
                ik_selected = selectedIK(oneIK);
                allIK.push_back(ik_selected);
            }

            // Assign each configuration to the corresponding joints
            std::vector<std::vector<double>> configs(numPoints);
            sensor_msgs::JointState my_joints;
            //std::stringstream sstr;
            for (int i = 0; i < allIK.size(); i++) {
                configs[i] =allIK[numPoints-i-1];
                    ROS_WARN_STREAM("1: "<<allIK[numPoints-i-1][0]<<" 2: "
                           <<allIK[numPoints-i-1][1]<<" 3: "
                             <<allIK[numPoints-i-1][2]<<" 4: "
                               <<allIK[numPoints-i-1][3]<<" 5: "
                                 <<allIK[numPoints-i-1][4]<<" 6: "
                                   <<allIK[numPoints-i-1][5]);
                    my_joints.name.resize(6);
                    my_joints.position.resize(6);
                    my_joints.position[0] = configs[i][0];
                    my_joints.position[1] = configs[i][1];
                    my_joints.position[2] = configs[i][2];
                    my_joints.position[3] = configs[i][3];
                    my_joints.position[4] = configs[i][4];
                    my_joints.position[5] = configs[i][5];
                    chesslab_setup::setConf setConf_srv;
                    setConf_srv.request.newConf = my_joints;
                    newConf_client.call(setConf_srv);
            }


            // Attacher node
            ros::ServiceClient serviceA = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

            // Create the objects for the service
            gazebo_ros_link_attacher::Attach::Request reqA;
            gazebo_ros_link_attacher::Attach::Response respA;

            // Prepare the srv data
            reqA.model_name_1 = req.name;
            reqA.link_name_1 = "link";
            reqA.model_name_2 = "team_A_arm";
            reqA.link_name_2 = "team_A_gripper_left_follower";

            // Call the service to attack the piece to the robot
            ros::service::waitForService("/link_attacher_node/attach", ros::Duration(5));
            bool successA = serviceA.call(reqA,respA);
            ROS_INFO_STREAM(successA);


            // Repeat the movement but going upwards
            for (int i = 0; i < allIK.size(); i++)  {
                configs[i] =allIK[i];
                    ROS_WARN_STREAM("1: "<<allIK[i][0]<<" 2: "
                           <<allIK[i][1]<<" 3: "
                             <<allIK[i][2]<<" 4: "
                               <<allIK[i][3]<<" 5: "
                                 <<allIK[i][4]<<" 6: "
                                   <<allIK[i][5]);

                    my_joints.name.resize(6);
                    my_joints.position.resize(6);
                    my_joints.position[0] = configs[i][0];
                    my_joints.position[1] = configs[i][1];
                    my_joints.position[2] = configs[i][2];
                    my_joints.position[3] = configs[i][3];
                    my_joints.position[4] = configs[i][4];
                    my_joints.position[5] = configs[i][5];
                    chesslab_setup::setConf setConf_srv;
                    setConf_srv.request.newConf = my_joints;
                    newConf_client.call(setConf_srv);
            }

            // Compute the ik of the steps in the destination
            std::vector<std::vector<double>> allIK_rev; //Contains all the configurations of the trajectory
            for(int i = 0; i<numPoints; ++i){
                oneIK = getIK(finalPose, numPoints, robotPositionHeight, saveMargin, i);
                ik_selected = selectedIK(oneIK);
                allIK_rev.push_back(ik_selected);
            }

            // Assign each configuration to the corresponding joints
            for (int i = 0; i < allIK.size(); i++) {
                configs[i] =allIK_rev[numPoints-i-1];
                    ROS_WARN_STREAM("1: "<<allIK[numPoints-i-1][0]<<" 2: "
                           <<allIK_rev[numPoints-i-1][1]<<" 3: "
                             <<allIK_rev[numPoints-i-1][2]<<" 4: "
                               <<allIK_rev[numPoints-i-1][3]<<" 5: "
                                 <<allIK_rev[numPoints-i-1][4]<<" 6: "
                                   <<allIK_rev[numPoints-i-1][5]);

                    my_joints.name.resize(6);
                    my_joints.position.resize(6);
                    my_joints.position[0] = configs[i][0];
                    my_joints.position[1] = configs[i][1];
                    my_joints.position[2] = configs[i][2];
                    my_joints.position[3] = configs[i][3];
                    my_joints.position[4] = configs[i][4];
                    my_joints.position[5] = configs[i][5];
                    chesslab_setup::setConf setConf_srv;
                    setConf_srv.request.newConf = my_joints;
                    newConf_client.call(setConf_srv);

            }


            // Dettacher node
            ros::ServiceClient serviceD = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

            // Create the objects for the service
            gazebo_ros_link_attacher::Attach::Request reqD;
            gazebo_ros_link_attacher::Attach::Response respD;

            // Prepare the srv data
            reqD.model_name_1 = req.name;
            reqD.link_name_1 = "link";
            reqD.model_name_2 = "team_A_arm";
            reqD.link_name_2 = "team_A_gripper_left_follower";

            // Call the service to dettach the piece
            ros::service::waitForService("/link_attacher_node/detach", ros::Duration(5));
            bool successD = serviceD.call(reqD,respD);
            ROS_WARN_STREAM(successD);


            // Repeat the steps upwards
            for (int i = 0; i < allIK.size(); i++)  {
                configs[i] =allIK_rev[i];
                    ROS_WARN_STREAM("1: "<<allIK_rev[i][0]<<" 2: "
                           <<allIK_rev[i][1]<<" 3: "
                             <<allIK_rev[i][2]<<" 4: "
                               <<allIK_rev[i][3]<<" 5: "
                                 <<allIK_rev[i][4]<<" 6: "
                                   <<allIK_rev[i][5]);

                    my_joints.name.resize(6);
                    my_joints.position.resize(6);
                    my_joints.position[0] = configs[i][0];
                    my_joints.position[1] = configs[i][1];
                    my_joints.position[2] = configs[i][2];
                    my_joints.position[3] = configs[i][3];
                    my_joints.position[4] = configs[i][4];
                    my_joints.position[5] = configs[i][5];
                    chesslab_setup::setConf setConf_srv;
                    setConf_srv.request.newConf = my_joints;
                    newConf_client.call(setConf_srv);

            }

// We tried implementing a linear trajectory but it did not work


        //    ///////////////////////////////////////////
        //    //Create the request and response objects for setting the traj
        //    chesslab_setup::set_point_to_point_trajectory::Request reqsetraj;
        //    chesslab_setup::set_point_to_point_trajectory::Response respsettraj;
        //
        //    //Set point to point trajectory
        //    // reqsetraj.goalPoint.joint_names.resize(6);
        //    // reqsetraj.goalPoint.joint_names[0] = "team_A_elbow_joint";
        //    // reqsetraj.goalPoint.joint_names[1] = "team_A_shoulder_lift_joint";
        //    // reqsetraj.goalPoint.joint_names[2] = "team_A_shoulder_pan_joint";
        //    // reqsetraj.goalPoint.joint_names[3] = "team_A_wrist_1_joint";
        //    // reqsetraj.goalPoint.joint_names[4] = "team_A_wrist_2_joint";
        //    // reqsetraj.goalPoint.joint_names[5] = "team_A_wrist_3_joint";
        //
        //    // Set the robot configurations in the points of the trajectory
        //    reqsetraj.goalPoint.joint_names.resize(6);
        //    reqsetraj.goalPoint.points.resize(numPoints);
        //    for (int i = 0; i < numPoints; ++i) {
        //       reqsetraj.goalPoint.points[i].positions.resize(6);
        //       reqsetraj.goalPoint.points[i].positions[0] = configs[i][2];
        //       reqsetraj.goalPoint.points[i].positions[1] = configs[i][1];
        //       reqsetraj.goalPoint.points[i].positions[2] = configs[i][0];
        //       reqsetraj.goalPoint.points[i].positions[3] = configs[i][3];
        //       reqsetraj.goalPoint.points[i].positions[4] = configs[i][4];
        //       reqsetraj.goalPoint.points[i].positions[5] = configs[i][5];
        //       reqsetraj.goalPoint.points[i].velocities.resize(6);
        //       reqsetraj.goalPoint.points[i].accelerations.resize(6);
        //       reqsetraj.goalPoint.points[i].time_from_start = ros::Duration(1+0.15*i);
        //    }
        //
        //
        //    //call the set_trajectory service
        //
        //    bool kpaso = this->client_settraj.call(reqsetraj,respsettraj);
        //
        //    ///////////////////////////////////////////
        //    //Create the request and response objects for moving the robot
        //    chesslab_setup::move_robot_trajectory::Request reqmovetraj;
        //    chesslab_setup::move_robot_trajectory::Response respmovetraj;
        //    reqmovetraj.trajduration = moveduration;
        //    ros::service::waitForService("move_robot_trajectory", ros::Duration(5));
        //
        //    ///////////////////////////////////////////
        //    //Create the request and response objects for moving the robot with limits
        //    chesslab_setup::move_robot_trajectory_safe::Request reqmovetrajsafe;
        //    chesslab_setup::move_robot_trajectory_safe::Response respmovetrajsafe;
        //    reqmovetrajsafe.trajduration = moveduration;
        //    reqmovetrajsafe.limits.resize(6);
        //    reqmovetrajsafe.limits[0] = -100000000000000.0; //xmin
        //    reqmovetrajsafe.limits[1] =  100000000000000.0; //xmax
        //    reqmovetrajsafe.limits[2] = -100000000000.0; //ymin
        //    reqmovetrajsafe.limits[3] =  100000000000.0; //ymax
        //    reqmovetrajsafe.limits[4] = 0.0000001; //zmin
        //    reqmovetrajsafe.limits[5] =  1000000000000.0; //zmax
        //    //ros::service::waitForService("move_robot_trajectorysafe", ros::Duration(5));
        //   //client_moverobottrajsafe.call(reqmovetrajsafe,respmovetrajsafe);
        //   // Move the robot
        //   client_moverobottraj.call(reqmovetraj,respmovetraj);

            return true;
        }


        std::vector<std::vector<double>> getIK(geometry_msgs::Pose desired_pose,  int numPoints, double robotPositionHeight, double saveMargin, int i){
            // Function to compute the inverse kinematics for a given movement.
            // IMPUTS:  desired pose to define x and y coordinates.
            //          numPoints, number of points the rectilinear movement is splitted
            //          robotPositionHeight, offset over the piece
            //          saveMargin, extra offset for safety
            // OUTPUT:  a set of inverse kinematics for a specific location

            // Local variables
            std::vector<std::vector<double>> ik_double;

            // srv data for the inverse kinematics service
            inversekin_srv.request.pose = desired_pose;

            // Set robot height
            inversekin_srv.request.pose.position.z = (robotPositionHeight - desired_pose.position.z - saveMargin)/numPoints*i + desired_pose.position.z + saveMargin + 0.14;

            // obtain the inverse kinematics
            inversekin_client.call(inversekin_srv);

            // Announce the height of the step
            ROS_WARN_STREAM("Robot Z: " <<
                inversekin_srv.request.pose.position.z);

            /*
            ROS_INFO_STREAM("Robot Pose Above the initial Piece: [" <<
                inversekin_srv.request.pose.position.x << ", " <<
                inversekin_srv.request.pose.position.y << ", " <<
                inversekin_srv.request.pose.position.z << ", " <<
                inversekin_srv.request.pose.orientation.x << ", " <<
                inversekin_srv.request.pose.orientation.y << ", " <<
                inversekin_srv.request.pose.orientation.z << ", " <<
                inversekin_srv.request.pose.orientation.w << "]");
*/
            //std::stringstream sstr;

            // Check if the inverse kinematics has a possible solution
            if(inversekin_srv.response.status){
                //sstr<<"The computed ik for the robot above the piece at point " << i <<" is:"<<std::endl;
                for(int i=0; i<inversekin_srv.response.ik_solution.size(); i++){
                    std::vector<double> ik_candidate;
                    //sstr << "[";
                    for(int j=0; j<5; j++){
                        ik_candidate.push_back(inversekin_srv.response.ik_solution[i].ik[j]);
                        //sstr << inversekin_srv.response.ik_solution[i].ik[j] <<", ";
                    }
                    ik_candidate.push_back(inversekin_srv.response.ik_solution[i].ik[5]);
                    //sstr << inversekin_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
                    ik_double.push_back(ik_candidate);
                }
                //ROS_INFO_STREAM(sstr.str());
            }
            else{ // return error if there is not possible solution
                ROS_INFO("Not able to compute the ik of the robot above the piece");
            }
            return ik_double;
        }


     // Select the best IK
    // We could avoid this function and calculate it inside the getIK function, but for clarity and scalability we have decided to do it this way.
    // We select based on the values of the joint configurations, although we could have choosen directly always the first ik because empirically we have seen that it is always the one that we are interested in
        std::vector<double> selectedIK(std::vector<std::vector<double>> oneIK){
            // Function to select the best inverse kinematics for the robot according to the requirements.
            // INPUT: the set of all possible inverse kinematics.
            // OUTPUT: the best configuration.

            // Local variables
            std::vector<double> ik_selected;
            std::stringstream sstr;
            bool selection = false;
            for(int row=0; row<oneIK.size(); row++)
            {
              if((oneIK[row][2] <=0 && oneIK[row][4] >= 0 && oneIK[row][0] >= 0) || (oneIK[row][2] >= 0 && oneIK[row][4] <= 0 && oneIK[row][0] >= 0))
                {
                      if(!ik_selected.empty())
                          if(std::abs(std::abs(oneIK[row][1])-3.14/2) < std::abs(std::abs(ik_selected[1])-3.14/2)){
                              ik_selected.clear();
                           selection = false;
                       }
                      if(!selection){
                          selection = true;
                          sstr<<"***The choosen ik for the robot is:"<<std::endl;
                           sstr << "[";
                          for(int j=0; j<=5; j++)
                          {
                               sstr << oneIK[row][j] << ", ";
                               ik_selected.push_back(oneIK[row][j]);
                           }
                          sstr << "]";
                            sstr<<"***"<<std::endl;
                      }

                }
            }
             ROS_INFO_STREAM(sstr.str());
            return ik_selected;
        }

        // Main loop
        void run(){
            ros::Rate rate(15);
            while(ros::ok()) {
               // ROS_INFO_ONCE("move_robot is running.");
                ros::spinOnce();
                rate.sleep();
            }
        }
};





int main(int argc, char **argv){
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "follow_traj_client");
    std::string name = ros::this_node::getName();
    move_robot_class node(name);

    node.run();
    return 0;
}

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <chesslab_setup/set_point_to_point_trajectory.h>
#include <chesslab_setup/set_goal_tolerances.h>
#include <chesslab_setup/move_robot_trajectory.h>
#include <chesslab_setup/move_robot_trajectory_safe.h>


//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

control_msgs::FollowJointTrajectoryGoal goal;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *robotClient;
bool cancelgoalactivated = false;
double xmin,xmax,ymin,ymax,zmin,zmax;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
geometry_msgs::TransformStamped transformStamped;
std::vector<double> currentjoints;
bool updated = false;

//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: error_code is %d", result->error_code);
  //error_string is not showing the message...
  //see http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html for the meaning of the error codes.
  //ROS_INFO_STREAM("Answer: error_string is "<< result->error_string);
}

//Callback function: Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

//Callback function: Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  //ROS_INFO("************: desired angles are (%f,%f)", feedback->desired.positions[0], feedback->desired.positions[1]);
  ROS_INFO("Got Feedback: current positions are (%f,%f,%f,%f,%f,%f)", feedback->actual.positions[0], feedback->actual.positions[1], feedback->actual.positions[2], feedback->actual.positions[3], feedback->actual.positions[4], feedback->actual.positions[5]);
  //ROS_INFO("              current velocities    are (%f,%f)", feedback->actual.velocities[0], feedback->actual.velocities[1]);
  //Acceleration feedback field is not filled, so do not print:
  //ROS_INFO("              accelerations.size() = %d", feedback->actual.accelerations.size());
  //ROS_INFO("              current accelerations are (%f,%f)", feedback->actual.accelerations[0], feedback->actual.accelerations[1]);

  /*if(fabs(feedback->actual.positions[0]-0.2) < 0.01){
    robotClient->cancelGoal();
  }
  */
  if(cancelgoalactivated){
    for(int i=0;i<3;i++){
        try{
            transformStamped = tfBuffer.lookupTransform("base_link","tool0", ros::Time(0));
        }
        catch (tf2::TransformException &ex){
            ROS_WARN("%d, %s",i+1, ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
        //ROS_INFO("%d Time %f", i, transformStamped.header.stamp.toSec());

        ROS_INFO("Got Feedback: current TCP positions are   (%f,%f, %f)", transformStamped.transform.translation.x, transformStamped.transform.translation.y,transformStamped.transform.translation.z);
        //ROS_INFO("Got Feedback: current TCP orientation are (%f,%f, %f, %f)", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

        if(transformStamped.transform.translation.x < xmin || transformStamped.transform.translation.x > xmax )
        {
            ROS_ERROR("Goal Cancelled:  x position %f is out of limits (%f,%f)",transformStamped.transform.translation.x, xmin,xmax);
            robotClient->cancelGoal();
        }
        else if(transformStamped.transform.translation.y < ymin || transformStamped.transform.translation.y > ymax )
        {
            ROS_ERROR("Goal Cancelled:  y position %f is out of limits (%f,%f)",transformStamped.transform.translation.y, ymin,ymax);
            robotClient->cancelGoal();
        }
        else if(transformStamped.transform.translation.z < zmin || transformStamped.transform.translation.z > zmax )
        {
            ROS_ERROR("Goal Cancelled:  z position %f is out0.3 of limits (%f,%f)",transformStamped.transform.translation.z, zmin,zmax);
            robotClient->cancelGoal();
        }
        break;
    }
  }
}


//Sends the goal to the FollowJointTrajectory action server and waits for the result for trajduration seconds
//If not able to reach the goal within timeout, it is cancelled
bool moveRobotTrajectory(
        chesslab_setup::move_robot_trajectory::Request &req,
        chesslab_setup::move_robot_trajectory::Response &resp)
{
  //Motions that does NOT allows preemption depending on the feedback
  cancelgoalactivated = false;
  //Set timestamp and send goal
  goal.trajectory.header.stamp = ros::Time::now();
  robotClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  //Wait for the action to return. Timeout set to the req.trajduration plus the goal time tolerance)
  bool finished_before_timeout = robotClient->waitForResult(ros::Duration(req.trajduration) + goal.goal_time_tolerance);

  //Get final state
  actionlib::SimpleClientGoalState state = robotClient->getState();
  if (finished_before_timeout) {
      //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
      //Reports SUCCEEDED if finished and goal reached
      ROS_INFO(" ***************** Robot action finished: %s  *****************",state.toString().c_str());
  } else {
      //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
      ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
      //Preempting task
      ROS_ERROR("I am going to preempt the task...");
      robotClient->cancelGoal();
  }
  return finished_before_timeout;
}

//Sends the goal to the FollowJointTrajectory action server and waits for the result for trajduration seconds
//If not able to reach the goal within timeout, it is cancelled
bool moveRobotTrajectorySafe(
        chesslab_setup::move_robot_trajectory_safe::Request &req,
        chesslab_setup::move_robot_trajectory_safe::Response &resp)
{
  //Motions that allows preemption depending on the feedback
  cancelgoalactivated = true;
  xmin = req.limits[0];
  xmax = req.limits[1];
  ymin = req.limits[2];
  ymax = req.limits[3];
  zmin = req.limits[4];
  zmax = req.limits[5];
  //Set timestamp and send goal
  goal.trajectory.header.stamp = ros::Time::now();
  robotClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  //Wait for the action to return. Timeout set to the req.trajduration plus the goal time tolerance)
  bool finished_before_timeout = robotClient->waitForResult(ros::Duration(req.trajduration) + goal.goal_time_tolerance);

  //Get final state
  actionlib::SimpleClientGoalState state = robotClient->getState();
  if (finished_before_timeout) {
      //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
      //Reports SUCCEEDED if finished and goal reached
      ROS_INFO(" ***************** Robot action finished: %s  *****************",state.toString().c_str());
  } else {
      //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
      ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
      //Preempting task
      ROS_ERROR("I am going to preempt the task...");
      robotClient->cancelGoal();
  }
  return finished_before_timeout;
}


//Service to set the trajectory
bool setPointToPointTrajectory(
        chesslab_setup::set_point_to_point_trajectory::Request &req,
        chesslab_setup::set_point_to_point_trajectory::Response &resp)
{
  ROS_INFO("Setting new trajectory from current configuration to goal configuration ");
  bool success = false;

  goal.trajectory.joint_names.resize(6);
  goal.trajectory.joint_names[0] = "team_A_shoulder_pan_joint";
  goal.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
  goal.trajectory.joint_names[2] = "team_A_elbow_joint";
  goal.trajectory.joint_names[3] = "team_A_wrist_1_joint";
  goal.trajectory.joint_names[4] = "team_A_wrist_2_joint";
  goal.trajectory.joint_names[5] = "team_A_wrist_3_joint";
/*
  goal.trajectory.joint_names.resize( req.joint_names.size() );
  for(int i=0; i < req.joint_names.size(); i++){
      goal.trajectory.joint_names[i] = req.joint_names[i];
  }
*/
  goal.trajectory.points.resize( req.goalPoint.points.size() );
  //goal point
  for(int i=0; i < req.goalPoint.points.size(); i++){
      uint vsize = req.goalPoint.points[i].positions.size();
      goal.trajectory.points[i].positions.resize( vsize );

      for(int j=0; j < req.goalPoint.points[i].positions.size(); j++){
          goal.trajectory.points[i].positions[j] = req.goalPoint.points[i].positions[j];
          // Only define velocities and accelerations for the last point of the trajectory
          if(i==vsize-1){
              goal.trajectory.points[i].velocities.resize( vsize );
              goal.trajectory.points[i].accelerations.resize( vsize );
              goal.trajectory.points[i].velocities[j] = 0.0;
              goal.trajectory.points[i].accelerations[j] = 0.0;
          }
      }
      ROS_ERROR("SI");
      goal.trajectory.points[i].time_from_start = req.goalPoint.points[i].time_from_start;

  }
}


//Service to set the trajectory
bool setGoalTolerances(
        chesslab_setup::set_goal_tolerances::Request &req,
        chesslab_setup::set_goal_tolerances::Response &resp){
        ROS_INFO("Setting trajectory tolerances...");

        goal.goal_tolerance.resize( req.jointGoalTolerances.size() );
        for(int i=0; i < req.jointGoalTolerances.size(); i++){
            goal.goal_tolerance[i].name = req.jointGoalTolerances[i].name;
            goal.goal_tolerance[i].position = req.jointGoalTolerances[i].position;
            goal.goal_tolerance[i].velocity = req.jointGoalTolerances[i].velocity;
            goal.goal_tolerance[i].acceleration = req.jointGoalTolerances[i].acceleration;
        }
        goal.goal_time_tolerance = req.goalTimeTolerance;

        return true;
}


int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "follow_traj_wrap_server");
  ros::NodeHandle nh;

  //Define simple action client and wail for server
  robotClient = new Client("/team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
  if(!robotClient->waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR(" *** action server not available *** ");
  };

  ros::ServiceServer server_settraj = nh.advertiseService("set_point_to_point_trajectory",&setPointToPointTrajectory);
  ros::ServiceServer server_setgoaltol = nh.advertiseService("set_goal_tolerances",&setGoalTolerances);
  ros::ServiceServer server_moverobottraj = nh.advertiseService("move_robot_trajectory",&moveRobotTrajectory);
  ros::ServiceServer server_moverobottrajsafe = nh.advertiseService("move_robot_trajectory_safe",&moveRobotTrajectorySafe);

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  ros::Rate rate(1);
  while(ros::ok()) {
    // Wait until it's time for another iteration.
    ros::spinOnce();
    rate.sleep();
  }

}

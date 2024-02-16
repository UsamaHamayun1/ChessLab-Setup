// Import libreries
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <string>
#include <cmath>

//The srv class for the service.
#include "chesslab_setup/poseOrientGrip.h"

// Global variables
tf2_ros::Buffer tfBuffer;
std::string actualName;     // To iterate over all pieces
std::string actualId;       // Actual aruco to compare them all to
geometry_msgs::TransformStamped stampedpose;
const std::string  namesArucos[32] = {"aruco_frame_201", "aruco_frame_202", "aruco_frame_203", "aruco_frame_204", "aruco_frame_205", "aruco_frame_206", "aruco_frame_207", "aruco_frame_208", "aruco_frame_301", "aruco_frame_302", "aruco_frame_303", "aruco_frame_304", "aruco_frame_305", "aruco_frame_306", "aruco_frame_307", "aruco_frame_308", "aruco_frame_209", "aruco_frame_210", "aruco_frame_211", "aruco_frame_212", "aruco_frame_213", "aruco_frame_214",  "aruco_frame_309", "aruco_frame_310", "aruco_frame_311", "aruco_frame_312", "aruco_frame_313", "aruco_frame_314", "aruco_frame_215", "aruco_frame_216", "aruco_frame_315", "aruco_frame_316"};

// Function to get the pose from the actual ID to all the rest
void eachPiece(std::string actualName, std::string actualId){
    // Local variables
    static geometry_msgs::TransformStamped transformStamped;

    // Get the pose of id1 wrt id2 
    for(int i=0; i<3; i++){
        try{
            transformStamped = tfBuffer.lookupTransform(actualId, actualName, ros::Time(0));//, ros::Duration(0.1));
        }
        catch (tf2::TransformException ex ){
            ROS_ERROR("%d - %s",i,ex.what());
            // Error
            continue;
        }
    }
    
    stampedpose = transformStamped;    
}

// Every time the service is called execute:
bool gripPose(chesslab_setup::poseOrientGrip::Request  &req, 
              chesslab_setup::poseOrientGrip::Response &res){
    // Local variables
    int Sx = 0;     // Number of small pieces in X direction
    int Mx = 0;     // Number of medium pieces in X direction
    int Bx = 0;     // Number of big pieces in X direction
    int Sy = 0;     // Number of small pieces in Y direction
    int My = 0;     // Number of medium pieces in Y direction
    int By = 0;     // Number of big pieces in Y direction
    double dist;    // Euclidian distance
    double xDif;    // Absolute distannce in X
    double yDif;    // Absolute distannce in Y
    
    // For each pair of arucos
    for (int piece = 0; piece<32; piece++){ 
        // Initialization
        dist = 0;
        actualId = req.poseChecked;         // Piece to know the surroundings of
        actualName = namesArucos[piece];    // Rest of the pieces to check
        
        // To avoid get the distance between the same piece
        if (actualName != actualId){
        
            // Get the tf each pair
        	eachPiece(actualId, actualName);
        	
        	// Compute distances
        	xDif = stampedpose.transform.translation.x;
        	yDif = stampedpose.transform.translation.y;
        	dist = std::sqrt(std::pow(abs(xDif),2) + std::pow(abs(yDif),2));
        	
        	    // If the pieces are near each other
                if ((0 < dist) && (dist <= 0.06)){
                    // check the direction of the distance
                    if (abs(xDif) <= abs(yDif)){
                        if (28 <= piece){   // The near piece is big in X
                            Bx = Bx+1;
                        }
                        else if ((16 <= piece) && (piece <= 27)){   // The near piece is medium in X
                            Mx = Mx+1;
                        }
                        else{   // The near piece is small in X
                            Sx = Sx+1;
                        }
                    }
                    else {
                        if (28 <= piece){   // The near piece is big in Y
                            By = By+1;
                        }
                        else if ((16 <= piece) && (piece <= 27)){   // The near piece is medium in Y
                            My = My+1;
                        }
                        else{   // The near piece is small in Y
                            Sy = Sy+1;
                        }
                    }
                } 
        }
    }
    
    // Look for the direction with smaller pieces and if same quantity, look for the medium pieces
    if (Sx < Sy){res.solu = true;}
    else if (Sy < Sx){res.solu = false;}
    else if (Mx < My){res.solu = true;}
    else if (My < Mx){res.solu = false;}
    else if (Bx < By){res.solu = true;}
    else {
    	res.solu = false;
    }
    return true;    
}


int main(int argc, char **argv){

    ros::init(argc, argv, "poseOrientGrip");
    ros::NodeHandle nh;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // The node provides a service to get the direction of the gripper lokking at the surroundings of the piece to pick
    ros::ServiceServer serviceGetGripPose = nh.advertiseService("chesslab_setup/poseOrientGrip", gripPose);
    

    tf2_ros::TransformListener tfListener(tfBuffer);
    ROS_INFO("ready!");
    ros::Rate rate(2);
    
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}

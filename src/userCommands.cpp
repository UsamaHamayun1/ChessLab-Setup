// Import libreries
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <unordered_map>

//The srv class for the service.
#include "chesslab_setup/userCommands.h"


// Global variables
std_msgs::String topicToPub;    // Variable that will be published
bool flag = false;              // Flag that enables to send the message just 1 time per user input

// Look up table to pass from pieces to aruco numbers
std::unordered_map<std::string, std::string> u = {
        {"PB1", "201 "}, // Pawn Black 1
        {"PB2", "202 "}, // Pawn Black 2
        {"PB3", "203 "}, // Pawn Black 3
        {"PB4", "204 "}, // Pawn Black 4
        {"PB5", "205 "}, // Pawn Black 5
        {"PB6", "206 "}, // Pawn Black 6
        {"PB7", "207 "}, // Pawn Black 7
        {"PB8", "208 "}, // Pawn Black 8
        {"RB1", "209 "}, // Rook Black 1
        {"RB2", "210 "}, // Rook Black 2
        {"KB1", "211 "}, // Knight Black 1
        {"KB2", "212 "}, // Knight Black 2
        {"BB1", "213 "}, // Bishop Black 1
        {"BB2", "214 "}, // Bishop Black 2
        {"QQB", "215 "}, // Queen Black 1
        {"KKB", "216 "}, // King Black 1
        {"KBL", "217 "}, // King Black Long castling
        {"KBS", "218 "}, // King Black Short castling
        {"PW1", "301 "}, // Pawn White 1
        {"PW2", "302 "}, // Pawn White 2
        {"PW3", "303 "}, // Pawn White 3
        {"PW4", "304 "}, // Pawn White 4
        {"PW5", "305 "}, // Pawn White 5
        {"PW6", "306 "}, // Pawn White 6
        {"PW7", "307 "}, // Pawn White 7
        {"PW8", "308 "}, // Pawn White 8
        {"RW1", "309 "}, // Rook White 1
        {"RW2", "310 "}, // Rook White 2
        {"KW1", "311 "}, // Knight White 1
        {"KW2", "312 "}, // Knight White 2
        {"BW1", "313 "}, // Bishop White 1
        {"BW2", "314 "}, // Bishop White 2
        {"QQW", "315 "}, // Queen White 1
        {"KKW", "316 "}, // King White 1
        {"KWL", "317 "}, // King White Long castling
        {"KWS", "318 "}  // King White Short castling
};

bool userC(chesslab_setup::userCommands::Request  &req, chesslab_setup::userCommands::Response &res)
{
    // Function callback when a new command is recibed via the service.
    // The input requested piece will be transfromed to a value using the look up table u. Then the flag will be enables and the message will be published.

    // Before saving the command, is tranformed using the look up table
    topicToPub.data = u[req.msg.substr(0, 3)] + req.msg.substr(4,2);

    // Enable the flag
    flag = true;

    // Return true has the message has been recibed
    res.returnMsg = true;
    return true;
}

int main(int argc, char **argv){
    // Main loop
    // This node contains the server of a serice and a publisher. The user inputs the desired data via terminal and it is processed.
    // The data is transformed to a value, pedending on the piece or the type of movement.
    // The input data consist in 5 characters. The first 3 corresponds to the pieces and the last two to the final position the piece should be.
    // The names of the pieces to make a request are forming a pattern(execept for the king and queen).
    // The first letter corresponds to the type.
    // P:= Pawns,
    // B:= Bishops,
    // K:= Knights,
    // R:= Rooks.
    // The second to the color.
    // W:= White
    // B:= Black
    // The last to which one in case there are multiple.
    // For example, to move the first black bishop to A3, the input must be "BB1 A3". The published message would be "213 A3"
    // The king and queen requires KK + color(for the kings) and QQ + color(for the queens).
    // To request for a castling move, use the comands K + color + direction, where direction is "S" for short and "L" for long.
    // Since with this code we input what color and in which direction, the 2 last characters of the input mult be filled with "--".
    // For example, for a long castling in black the input must be "KBL --".
    // In this cases, the published message corresponds to a pre-defined cases: "217", "218", "317" and "318" for "KBL", "KBS", "KWL" and "KWS" respectively.


    // Initialization of the node
    ros::init(argc, argv, "userCommandsNode");
    ros::NodeHandle nh;

    // The node provides a service to publish the user commnds to the rest of the nodes
    ros::ServiceServer serviceGetPiecePose = nh.advertiseService("chesslab_setup/userCommands", userC);

    // Publiser of the commands.
    ros::Publisher pub = nh.advertise<std_msgs::String>("chesslab_setup/commands", 1000);
    topicToPub.data = "No commands yet";

    ROS_INFO("ready!");
    ros::Rate rate(2);

    while (ros::ok()) {
        // Wait until a new message are to the service
        if (flag){
            // Announce the new message
            ROS_INFO_STREAM("The new command is:");
            ROS_INFO_STREAM(topicToPub.data);

            // Publish the new message
            pub.publish(topicToPub);

            // Avoid publishing the message 2 times. Turn flag off.
            flag = false;
        }
        ros::spinOnce();
    }
    return 0;
}

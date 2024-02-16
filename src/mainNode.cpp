// Import libreries
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <unordered_map>


//The srv class for the services.
#include "chesslab_setup/getpiecepose.h"
#include "chesslab_setup/pickAndPlace.h"

// Global variables
bool flag1 = false;			                // Flags to avoid repetitions in the while
bool flag2 = false;
bool occupaidPiece = false;		            // To know if the destination is full to do 1 or 2 pick and places
bool orientGrip = false;		            // To define the orientation of the gripper. True to the x orientation, false otherwise
std::string movement;			            // Input command string from the userCommand node
std::string removeCommand;		            // Command related to the pice that it was killed
std::string moveCommand;		            // Command related to the movement of the piece that kills
std::string pieceIDString;		            // String of the number of the aruco that is requested to be moved
std::string pieceIDStringDelete;            // Piece to be moved to the trash
std::string pieceIDStringKing;	            // Similar but king and rook names when castling
std::string pieceIDStringRook;
std::string targetPosition;		            // Target position of the piece asked to be moved
std::string targetPositionKing;	            // Similar but king and rook positions when castling
std::string targetPositionRook;
geometry_msgs::TransformStamped chessPlace; // Position to place the picked pieces
geometry_msgs::Pose init1;                  // Initial position to pick the piece
geometry_msgs::Pose init2;                  // In case of killing a piece, we need 2 positions
geometry_msgs::Pose finalPose1;             // Final position to place the piece
geometry_msgs::Pose finalPose2;             // In case of killing a piece, we need 2 positions    
const int SIZE = 8;                         // Size of the chessboard
int pieceID;                                // Value of the aruco of the piece requested to be moved
char colIndex, colIndexRook, colIndexKing;  // Index(final and source) of the columns and rows of the requested pieces, similar with king and rook when castling
int rowIndex, rowIndexRook, rowIndexKing;
int sourceRowIndex, sourceColIndex, sourceRowIndexRook, sourceColIndexRook, sourceRowIndexKing, sourceColIndexKing;

ros::ServiceClient serviceGetPiecePose;     	// Service getpiecepose
ros::ServiceClient serviceFollowTrajClient;     // Service followTrajClient

int array2d[8][8] = { // Initial display of the scenario and the pieces positions
    {209, 211, 213, 215, 216, 214, 212, 210},
    {201, 202, 203, 204, 205, 206, 207, 208},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {308, 307, 306, 305, 304, 303, 302, 301},
    {310, 312, 314, 315, 316, 313, 311, 309},
};

// Map to know the size of the piece when checked the surrouning pieces using the map
std::unordered_map<int, int> u = {
        {0, 201}, // Pawn Black 1
        {1, 202}, // Pawn Black 2
        {2, 203}, // Pawn Black 3
        {3, 204}, // Pawn Black 4
        {4, 205}, // Pawn Black 5
        {5, 206}, // Pawn Black 6
        {6, 207}, // Pawn Black 7
        {7, 208}, // Pawn Black 8
        {8, 301}, // Pawn White 1
        {9, 302}, // Pawn White 2
        {10, 303}, // Pawn White 3
        {11, 304}, // Pawn White 4
        {12, 305}, // Pawn White 5
        {13, 306}, // Pawn White 6
        {14, 307}, // Pawn White 7
        {15, 308}, // Pawn White 8
        {16, 209}, // Rook Black 1
        {17, 210}, // Rook Black 2
        {18, 309}, // Rook White 1
        {19, 310}, // Rook White 2
        {20, 211}, // Knight Black 1
        {21, 212}, // Knight Black 2
        {22, 311}, // Knight White 1
        {23, 312}, // Knight White 2
        {24, 213}, // Bishop Black 1
        {25, 214}, // Bishop Black 2
        {26, 313}, // Bishop White 1
        {27, 314}, // Bishop White 2
        {28, 215}, // Queen Black 1
        {29, 315}, // Queen White 1
        {30, 216}, // King Black 1
        {31, 316}, // King White 1
};

// Map to send the name of the piece we want to attach or dettach
std::unordered_map<std::string, std::string> v = {
        {"201", "pawnB1"}, // Pawn Black 1
        {"202", "pawnB2"}, // Pawn Black 2
        {"203", "pawnB3"}, // Pawn Black 3
        {"204", "pawnB4"}, // Pawn Black 4
        {"205", "pawnB5"}, // Pawn Black 5
        {"206", "pawnB6"}, // Pawn Black 6
        {"207", "pawnB7"}, // Pawn Black 7
        {"208", "pawnB8"}, // Pawn Black 8
        {"209", "rookB1"}, // Rook Black 1
        {"210", "rookB2"}, // Rook Black 2
        {"211", "knightB1"}, // Knight Black 1
        {"212", "knightB2"}, // Knight Black 2
        {"213", "bishopB1"}, // Bishop Black 1
        {"214", "bishopB2"}, // Bishop Black 2
        {"215", "queenB"}, // Queen Black 1
        {"216", "kingB"}, // King Black 1
        {"301", "pawnW1"}, // Pawn White 1
        {"302", "pawnW2"}, // Pawn White 2
        {"303", "pawnW3"}, // Pawn White 3
        {"304", "pawnW4"}, // Pawn White 4
        {"305", "pawnW5"}, // Pawn White 5
        {"306", "pawnW6"}, // Pawn White 6
        {"307", "pawnW7"}, // Pawn White 7
        {"308", "pawnW8"}, // Pawn White 8
        {"309", "rookW1"}, // Rook White 1
        {"310", "rookW2"}, // Rook White 2
        {"311", "knightW1"}, // Knight White 1
        {"312", "knightW2"}, // Knight White 2
        {"313", "bishopW1"}, // Bishop White 1
        {"314", "bishopW2"}, // Bishop White 2
        {"315", "queenW"}, // Queen White 1
        {"316", "kingW"}, // King White 1
};

void chessToPose(std::string finalPoseName){
    // Function to pass from commands to coordinates.
    // INPUT: string of the pose we want the piece to finish in. 
    // This pose need to be in form "Letter+number" where letter corresponds to the columns of the chessboard and letter to the row.
    // OUTPUT: global variable tf 'chessPlace'. The variable include the position in x and y of the pose requested with respect to the base of the robot. z and orientation is set to 0.
    //
    // Example: chessToPose("D1")  -->  chessPlace = x:= -0.025
    //                                               y:= -0.545
    //                                               z:= 0
    //                                               Ox:= 0
    //                                               Oy:= 0   
    //                                               Oz:= 0
    //                                               Ow:= 0
    
    // Local variables, size of the square of the board and the middle value
    double sz = 0.05;
    double sm = sz/2;
    
    // Strings to numbers 
    int row = int(finalPoseName[1])-48;
    std::string colStr = finalPoseName.substr(0,1);
    int col;
    
    // Transform the letters to number of columns
    if (colStr == "A"){col = 1;}
    else if (colStr == "B"){col = 2;}
    else if (colStr == "C"){col = 3;}
    else if (colStr == "D"){col = 4;}
    else if (colStr == "E"){col = 5;}
    else if (colStr == "F"){col = 6;}
    else if (colStr == "G"){col = 7;}
    else {col = 8;}
    
    // Get the position of the board of the desired final movement
    double xPos = (col-1)*sz+(-3*sz-sm)+(-0);
    double yPos = (row-1)*sz+(-3*sz-sm)+(-0.37);
    
    chessPlace.transform.translation.x = xPos;
    chessPlace.transform.translation.y = yPos;
}

void messageReceived(const std_msgs::String &msg) {
    // Function callback when a new command is recibed via publisher.
    // The value is stored, readed and processed to extract the piece value and the position where it should move.
    // The strings recibed "217", "218", "317", "318" corresponds to the castling movement, black long, black short, white long and white short respectivelly.
    // This function returns the global variables IDString, pieceIDString, targetPosition when a no-castling movement is required. 
    // The int variable IDString corresponds to the piece to move. Similarly, the variable pieceIDString returns the value inside a string. 
    // The string targetPosition corresponds to the place the piece should be placed.
    //
    // When a castling is done, the variables pieceIDStringRook, targetPositionKing, targetPositionRook are pre-defined to the correct values so no code is needed.
    // Also in those cases, the indices used in the planning module are defined.
    
    // Read the input msg
    movement = msg.data;
    
    // Check if we are not in a castling move
    if ((movement.substr(0,3) != "217") && (movement.substr(0,3) != "218") && (movement.substr(0,3) != "317") && (movement.substr(0,3) != "318")){
        // Announce the new command and enable the movement section on the main loop
        flag1 = true;
        ROS_INFO_STREAM("New command recibed:");
        ROS_INFO_STREAM(movement);

        // Extracting targetPosition and pieceID from the movement string
        size_t delimiterPos = movement.find(' ');
        if (delimiterPos != std::string::npos) {
            pieceIDString = movement.substr(0, delimiterPos);
            targetPosition = movement.substr(delimiterPos + 1);
            try {
                pieceID = std::stoi(pieceIDString);
            } 
            catch (const std::invalid_argument& e) {
                ROS_ERROR_STREAM("Invalid piece ID: " << pieceIDString);
                // Handle the error condition if the piece ID is not a valid integer
                return;
            }
        }
        else {
            ROS_ERROR_STREAM("Invalid movement string: " << movement);
            // Handle the error condition if the movement string doesn't contain a delimiter
            return;
        }
    }
    
    // If we are in a castling movement...
    else{
           
        // Announce the new command
        ROS_INFO_STREAM("New command recibed:");
        ROS_INFO_STREAM(movement);
        
        // Charge the pre-defined variables depending on the type of castling
        if (movement.substr(0,1) == "2"){ // Moving Black
            pieceIDStringKing = "216";
            sourceRowIndexKing = 7; 
            sourceColIndexKing = 4;
            if(movement.substr(2,1) == "7"){ // Long castling
                pieceIDStringRook = "209";
                targetPositionKing = "C8";
                targetPositionRook = "D8";
                sourceRowIndexRook = 7; 
                sourceColIndexRook = 0;
                rowIndexRook = 7; 
                colIndexRook = 3;
                rowIndexKing = 7; 
                colIndexKing = 2;
            }
            else{ // Short castling
                pieceIDStringRook = "210";
                targetPositionKing = "G8";
                targetPositionRook = "F8";
                sourceRowIndexRook = 7; 
                sourceColIndexRook = 7;
                rowIndexRook = 7; 
                colIndexRook = 5;
                rowIndexKing = 7; 
                colIndexKing = 6;
            }
        }
        else{ // Moving White
            pieceIDStringKing = "316";
            sourceRowIndexKing = 0; 
            sourceColIndexKing = 4;
            if(movement.substr(2,1) == "7"){ // Long castling
                pieceIDStringRook = "310";
                targetPositionKing = "C1";
                targetPositionRook = "D1";
                sourceRowIndexRook = 0; 
                sourceColIndexRook = 0;
                rowIndexRook = 0; 
                colIndexRook = 3;
                rowIndexKing = 0; 
                colIndexKing = 2;
            }
            else{ // Short castling
                pieceIDStringRook = "309";
                targetPositionKing = "G1";
                targetPositionRook = "F1";
                sourceRowIndexRook = 0; 
                sourceColIndexRook = 7;
                rowIndexRook = 0; 
                colIndexRook = 5;
                rowIndexKing = 0; 
                colIndexKing = 6;
            }
        }
        
        // If one of the final poses is occupied, the movement is not done
        if(((array2d[7-rowIndexRook][colIndexRook] == 0) && (array2d[7-rowIndexKing][colIndexKing] == 0) && (movement.substr(2,1) != "7")) ||
            ((array2d[7-rowIndexRook][colIndexRook] == 0) && (array2d[7-rowIndexKing][colIndexKing] == 0) && (array2d[7-0][1] == 0) && (movement.substr(2,1) == "7"))){
            // Enable the castling movement in the main node
            flag2 = true;
            
            // In this part of the code the planning module is not used so the scene is actualized directly
            array2d[7-rowIndexKing][colIndexKing] = std::stoi(pieceIDStringKing);
            array2d[7-sourceRowIndexKing][sourceColIndexKing] = 0;
            array2d[7-rowIndexRook][colIndexRook] = std::stoi(pieceIDStringRook);
            array2d[7-sourceRowIndexRook][sourceColIndexRook] = 0;    
        }
        else{ // return error if the movement is not possible
            ROS_INFO_STREAM("One of the movements is not possible");
        }
    }
}

void gripPose(int row, int col, int mat[][8]){
    // Function to check the position the gripper must have when picking and placing a robot to not collide with close pieces.
    // It checks the surrounding pieces and according to its heigh, is selects the best position. 
    // The functions preferes multiple small pieces over a single big pieces.
    // INPUT: row: int of the row of the position we want to check, lastest row = 0.
    //        col: int of the column of the position we want to check, leftest column = 0.
    //        mat: 2d array of the actual scenario. The first dimension corresponds to the rows and the second to the columns.
    // OUTPUT: global boolean variable orientGrip. Returns true when the pose of the gripper should be in the x direction, false otherwise.
    //
    // EXAMPLE: gripPose(2,2, mat) (which corresponds to the position where piece 307 is)
    //                             where mat =  {209, 211, 213, 215, 216, 214, 212, 210},
    //                                          {201, 202, 203, 204, 205, 206, 207, 208},
    //                                          {0, 0, 0, 0, 0, 0, 0, 0},
    //                                          {0, 0, 0, 0, 0, 0, 0, 0},
    //                                          {0, 0, 0, 0, 0, 0, 0, 0},
    //                                          {0, 0, 0, 0, 0, 0, 0, 0},
    //                                          {308, 307, 306, 305, 304, 303, 302, 301},
    //                                          {310, 312, 314, 315, 316, 313, 311, 309},
    //          orientGrip = true
    
    // Local variables
    int valTop = mat[7-(row+1)][col];     // Piece at the top of the position given
    int valBot = mat[7-(row-1)][col];     // Piece at the bottom of the position given
    int valLeft = mat[7-row][(col-1)];    // Piece at the left of the position given
    int valRight = mat[7-row][(col+1)];   // Piece at the right of the position given
    int Sx = 0;     // Number of small pieces in X direction
    int Mx = 0;     // Number of medium pieces in X direction
    int Bx = 0;     // Number of big pieces in X direction
    int Sy = 0;     // Number of small pieces in Y direction
    int My = 0;     // Number of medium pieces in Y direction
    int By = 0;     // Number of big pieces in Y direction
    
    // Check when the position is at the limits of the matrix.
    if (row == 0){valBot = 0;}
    else if (row == 7){valTop = 0;}
    if (col == 0){valLeft = 0;}
    else if (col == 7){valRight = 0;}
    
    // Check which pieces are in those 4 locations. 
    // From 0 to 16 are small pieces(pawns). 
    // From 16 to 27 are medium(bishop, knight, rook). 
    // 28 or higher are big (king and queen).
    for (int piece = 0; piece<32; piece++){
        // Check which piece is at the top or at the bottom
        if ((valTop == u[piece]) || (valBot == u[piece])){
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
        // Check which piece is at the left or at the right
        else if ((valLeft == u[piece]) || (valRight == u[piece])){
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
    // Look for the direction with smaller pieces and if there are the same, look for the next size pieces
    if (Sx < Sy){orientGrip = true;}
    else if (Sy < Sx){orientGrip = false;}
    else if (Mx < My){orientGrip = true;}
    else if (My < Mx){orientGrip = false;}
    else if (Bx < By){orientGrip = true;}
    else {
    	orientGrip = false;
    }
    // Announce the orientation
    ROS_INFO_STREAM("Orientation is " << orientGrip);
}

void displayMatrix(int array2d[][8]) {
    // Function to display de actual scene of the game stored in a matrix.
    // INPUT: 2d array of the actual scenario. The first dimension corresponds to the rows and the second to the columns.
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            std::cout << array2d[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
void computeInformation (bool trash){
    // Function to compute the information related with the position of the piece of the pick and place. 
    // INPUT: boolean variable to decide if the final move is placed in the trash.
    // OUTPUT: global variables init1 and finalPose1 that corresponds to the initial and final positions of the pieces/locations specified in pieceIDString/targetPosition.
    
    // Local variable
    std::string initPiece;
    
    // Create the objects of the service getpiecepose
    chesslab_setup::getpiecepose::Request req;
    chesslab_setup::getpiecepose::Response resp;
    
    // If the final position is the trash, the piece ID is the one of the piece to delete
    if (trash){
        initPiece = pieceIDStringDelete;
    }
    else{ // If not, the piece is the one to move to the target
        initPiece = pieceIDString;
    }
    
    // Request the position of the piece
    req.id1 = "aruco_frame_" + initPiece;
    req.id2 = "team_A_base";
        
    // Call the service to get the position of the piece
    ros::service::waitForService("chesslab_setup/getpiecepose", ros::Duration(5));
    bool success1 = serviceGetPiecePose.call(req,resp);
        
    // Store the position in a variable pose "init1"
    init1.position.x = resp.stampedpose.transform.translation.x;
    init1.position.y = resp.stampedpose.transform.translation.y;
    init1.position.z = resp.stampedpose.transform.translation.z;
    
    // Get the ideal final position of the piece
    chessToPose(targetPosition); 
                
    // Store the position in a variable pose "finalPose1"
    finalPose1.position.x = chessPlace.transform.translation.x;
    finalPose1.position.y = chessPlace.transform.translation.y;
    finalPose1.position.z = resp.stampedpose.transform.translation.z;
    
    // If we are moving to the trash, the final pose is always the same
    if (trash){    
        finalPose1.position.x = -25;
        finalPose1.position.y = 0;
    }
}

void sentToRobot(std::string name){
    // Create the objects of the service
    chesslab_setup::pickAndPlace::Request req;
    chesslab_setup::pickAndPlace::Response resp;
    
    // Prepare the positions for the pick and place
    req.poseInitial = init1;
    req.poseFinal = finalPose1;
    
    // Compute the gripper orientation for the pick
    gripPose(sourceRowIndex, sourceColIndex, array2d);
    req.orientationInitial;
    
    // Compute the gripper orientation for the place
    gripPose(rowIndex, colIndex, array2d);
    req.orientationFinal;
    
    req.name = name;
    
    // Call the service to get the position of the piece
    ros::service::waitForService("follow_traj_client/pickAndPlace", ros::Duration(5));
    bool success1 = serviceFollowTrajClient.call(req,resp);
}

int main( int argc, char** argv ){
    // Main loop 
    // The currend node uses as a client the service "getpiecepose" to obtain the position of a piece given the id and a reference frame, usually the base of the robot.
    // The current node subscribes to the "userCommands" node to obtain the orders the user inputs to the program.
    // The nodes uses a matrix as a planning module that stores the position of the pieces of the game. 
    // When a new command is entered, the algorithm computes the required movements and sends them to the sensing module, that gets the positions and other information required for the movement.
    // There exist 2 scenarios, a normal movement and a castling. Depending on what the user request, the corresponding flags enable the different parts of the node.
    
    // Initialization of the node
    ros::init(argc, argv, "mainNode");
    ros::NodeHandle nh;
    
    // Getpiecepose service client 
    serviceGetPiecePose = nh.serviceClient<chesslab_setup::getpiecepose>("chesslab_setup/getpiecepose");
    
    // Subscriber to the userCommand node that publishes the user instructions
    ros::Subscriber sub = nh.subscribe("chesslab_setup/commands", 1000, &messageReceived);
    
    // Pick and place service
    serviceFollowTrajClient = nh.serviceClient<chesslab_setup::pickAndPlace>("follow_traj_client/pickAndPlace");

    // Set the initial scene
    if(argc!=1 && strcmp(argv[1], "initialize") == 0){
        std::cout<<"Initializing SCENE"<<std::endl;
        ros::service::waitForService("/chesslab_setup/resetscene");
        ros::ServiceClient resetscene_client = nh.serviceClient<std_srvs::Empty>("/chesslab_setup/resetscene");
        std_srvs::Empty resetscene_srv;
        resetscene_client.call(resetscene_srv);
    }

    while (ros::ok()){
        // Prepare the indices for the planning module 
        colIndex = targetPosition[0]-'A';
        rowIndex = targetPosition[1] - '1';

        // Case of a normal movement
        if (flag1) {
            // Check if the position we want to move the piece to is empty of is already occupied by another piece.
            if (array2d[7-rowIndex][colIndex] != 0) { // The place is occupied
                std::cout<<"Target postion is already occupied. Removing the piece." << std::endl;

                // Generate the commmand string to remove the exiting piece to the trash (outside the board)
                std::ostringstream command;
                command << "aruco_frame_" << array2d[7-rowIndex][colIndex] << " TR";
                removeCommand = command.str();
                
                // Generate The command string for moving the new piece
                std::ostringstream newCommand;
                newCommand << "aruco_frame_" << pieceID << " " << targetPosition;
                moveCommand = newCommand.str();
                
                // Find the current position of the piece
                for (int i = 0; i < SIZE; ++i) {
                    for (int j = 0; j < SIZE; ++j) {
                        if (array2d[i][j] == pieceID) {
                            sourceRowIndex = 7-i;
                            sourceColIndex = j;
                            break;
                        }
                    }
                }
                
                // Save the piece it will be moved to the trash
                pieceIDStringDelete = std::to_string(array2d[7-rowIndex][colIndex]);
                
                // Move the piece to the target position
                array2d[7-rowIndex][colIndex] = pieceID;
                
                // The past position is now free
                array2d[7-sourceRowIndex][sourceColIndex] = 0;

                // Announce the final movements
                std::cout << "Remove Command: " << removeCommand << std::endl;
                std::cout << "Move Command: " << moveCommand << std::endl;
                
                // Prepare the case for a double pick and place
                occupaidPiece = true;

            } 
            else { // The place is free
                std::cout << "Target position is not occupied. Moving the piece." << std::endl;
                
                // Find the current position of the piece
                for (int i = 0; i < SIZE; ++i) {
                    for (int j = 0; j < SIZE; ++j) {
                        if (array2d[i][j] == pieceID) {
                            sourceRowIndex = 7-i;
                            sourceColIndex = j;
                            break;
                        }
                    }
                }

                // Move the piece to the target position
                array2d[7-rowIndex][colIndex] = pieceID;
                
                // The past position is now free
                array2d[7-sourceRowIndex][sourceColIndex] = 0;

                // Announce the final movement
                std::cout << "Move Command: aruco_frame_" << pieceID << " " << targetPosition << std::endl;
                
                // Prepare the case for a single pick and place
                occupaidPiece = false;
            }

            // Case for the double pick and place
        	if (occupaidPiece){
        	    // Compute the information of the positions
                computeInformation(true);     
                
                // Announce the serie of movements
                ROS_INFO_STREAM("Take " << "aruco_frame_" + pieceIDStringDelete << " from ");
                ROS_INFO_STREAM(init1);
                ROS_INFO_STREAM("and move to (Trash)");
                ROS_INFO_STREAM(finalPose1);
                
                // Sent the information to the action manager
                sentToRobot(v[pieceIDStringDelete]);
                
                // In this case we needed to make 2 movements, the first one is to take the occupying piece to trash (already did). 
                // The second one is to move the original requested piece to the desired position. this second movement is done now.
                
                // Compute the information of the positions
                computeInformation (false); 
                
                // Announce the serie of movements
                ROS_INFO_STREAM("Take " << "aruco_frame_" + pieceIDString << " from ");
                ROS_INFO_STREAM(init1);
                ROS_INFO_STREAM("and move to " << targetPosition);
                ROS_INFO_STREAM(finalPose1);
                
                // Sent the information to the action manager
                sentToRobot(v[pieceIDString]);
                
                // Once this movement is done, the flag turns off waiting for a new command to arrive
                flag1 = false;
            }
            
            // Similar but without the trash movement. In this case only 1 pick and place is done since the final position is free.
            else if (not(occupaidPiece)){
                // Compute the information of the positions
                computeInformation(false);               
                
                // Announce the movement
                ROS_INFO_STREAM("Take " << "aruco_frame_" + pieceIDString << " from ");
                ROS_INFO_STREAM(init1);
                ROS_INFO_STREAM("and move to " << targetPosition);
                ROS_INFO_STREAM(finalPose1);
                
                // Sent the information to the action manager
                sentToRobot(v[pieceIDString]);
                
                // Once this movement is done, the flag turns off waiting for a new command to arrive
                flag1 = false; 
            }
        displayMatrix(array2d);
        }
        
        // Case of a castling movement
        if (flag2){
            // In this case, the target and the pieceID is the king information 
            pieceIDString = pieceIDStringKing;
            targetPosition = targetPositionKing;
            computeInformation(false);
                      
            // Announce the king movement
            ROS_INFO_STREAM("Take " << "aruco_frame_" + pieceIDString << " from ");
            ROS_INFO_STREAM(init1);
            ROS_INFO_STREAM("and move to " << targetPosition);
            ROS_INFO_STREAM(finalPose1);
            
            // In this case, the target and the pieceID is the king information
            sourceRowIndex = sourceRowIndexKing;
            sourceColIndex = sourceColIndexKing;
            rowIndex = rowIndexKing;
            colIndex = colIndexKing;
            
            // Sent the information to the action manager
            sentToRobot(v[pieceIDString]);
            
            // In this case, the target and the pieceID is the rook information 
            pieceIDString = pieceIDStringRook;
            targetPosition = targetPositionRook;
            computeInformation(false);
            
            // Announce the rook movement
            ROS_INFO_STREAM("Take " << "aruco_frame_" + pieceIDString << " from ");
            ROS_INFO_STREAM(init2);
            ROS_INFO_STREAM("and move to " << targetPosition);
            ROS_INFO_STREAM(finalPose2);
            
            // In this case, the target and the pieceID is the rook information
            sourceRowIndex = sourceRowIndexRook;
            sourceColIndex = sourceColIndexRook;
            rowIndex = rowIndexRook;
            colIndex = colIndexRook;
            
            // Sent the information to the action manager
            sentToRobot(v[pieceIDString]);

            // Once this movement is done, the flag turns off waiting for a new command to arrive
            flag2 = false;
            displayMatrix(array2d);
        }
    ros::spinOnce();
    }                   
}

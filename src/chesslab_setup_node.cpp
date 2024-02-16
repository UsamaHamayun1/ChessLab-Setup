#include <map>
#include <sstream>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

#include <downward_ros/Plan.h>
#include <ur3ik/UR3IK.h>

#include <kautham/OpenProblem.h>
#include <kautham/CheckCollision.h>
#include <kautham/SetRobotsConfig.h>
#include <kautham/ObsPos.h>
#include <kautham/AttachObstacle2RobotLink.h>
#include <kautham/DetachObstacle.h>

/*
#include <kautham/SetQuery.h>
#include <kautham/GetPath.h>
#include <kautham/ObsPos.h>
#include <kautham/SetRobotsConfig.h>
#include <kautham/RemoveObstacle.h>
#include <kautham/SetInit.h>
#include <kautham/SetRobControls.h>
#include <kautham/FindIK.h>
*/

#include "chesslab_setup/setrobconf.h"
#include "chesslab_setup/setobjpose.h"
#include "chesslab_setup/attachobs2robot.h"
#include "chesslab_setup/dettachobs.h"
#include "chesslab_setup/ffplan.h"
#include "chesslab_setup/ik.h"


using namespace std;

std::vector<float> conf(14);
std::map<unsigned int, unsigned int> aruco2rviz_map;
std::map<unsigned int, unsigned int> rviz2aruco_map;
std::map<unsigned int, std::string> aruco2kautham_map;
std::map<std::string, unsigned int> kautham2aruco_map;
std::map<unsigned int, std::string> aruco2ff_map;
std::map<std::string,unsigned int> ff2aruco_map;

int attached  = -1;

tf2_ros::Buffer tfBuffer;


class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    std::string kthname;
    std::string frame_id;
    std::string name;
    geometry_msgs::Pose pose;
    double scale_x;
    double scale_y;
    double scale_z;
    double r;
    double g;
    double b;
    bool use_embedded_materials;
public:
    inline void setObjPath(std::string p) {objPath=p;}
    inline std::string getObjPath() {return objPath;}

    inline unsigned int getArucoID() {return id;}
    inline std::string getKauthamName() {return kthname;}
    inline std::string get_frame_id() {return frame_id;}
    inline std::string getname() {return name;}
    inline double getx() {return pose.position.x;}
    inline double gety() {return pose.position.y;}
    inline double getz() {return pose.position.z;}
    inline double getqx() {return pose.orientation.x;}
    inline double getqy() {return pose.orientation.y;}
    inline double getqz() {return pose.orientation.z;}
    inline double getqw() {return pose.orientation.w;}
    inline bool get_use_embeded_materials() {return use_embedded_materials;}
    inline double getr() {return r;}
    inline double getg() {return g;}
    inline double getb() {return b;}
    inline double getscale_x() {return scale_x;}
    inline double getscale_y() {return scale_y;}
    inline double getscale_z() {return scale_z;}
    inline geometry_msgs::Pose getPose() {return pose;}

    inline void setArucoID(unsigned int i) {id=i;}
    inline void setKauthamName(std::string name) {kthname=name;}
    inline void set_frame_id(std::string s) {frame_id=s;}
    inline void setname(std::string s) {name=s;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline void setr(double v) {r=v;}
    inline void setg(double v) {g=v;}
    inline void setb(double v) {b=v;}
    inline void setscale_x(double v) {scale_x=v;}
    inline void setscale_y(double v) {scale_y=v;}
    inline void setscale_z(double v) {scale_z=v;}
    inline void set_use_embedded_materials(bool t) {use_embedded_materials = t;}
};
std::vector<ObjectInfo> objects;


void SetChessWorld()
{
    ObjectInfo obj;
    double chessboard_height = 0.004;
    double short_height = 0.04; //pawns
    double middle_height = 0.06; //rook, knights and bishops
    double tall_height = 0.08; //queen, king

    //Pieces will be located w.r.t /chess_frame located at the center of the board onthe top surface

    //object id set top 100.
    //aruco markers on the board corners (100 to 103)

    objects.clear();

    //flat_chessboard dimension A2x0.004 (scale fixed such that it is obtained from a 0.05x0.05x0.05 cube)
    obj.setObjPath("package://chesslab_setup/models/flat_chessboard/meshes/flat_chessboard.dae");
    obj.setArucoID(100);
    obj.setKauthamName("flat_chessboard");
    obj.setname("FLAT_CHESSBOARD");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.009);
    obj.sety(0.0);
    obj.setz(-0.002); //reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/2.0));
    obj.setqw(cos(M_PI/2.0));
    obj.setscale_x(1.0); //59.4/5.0
    obj.setscale_y(1.0);//42.0/5.0
    obj.setscale_z(1.0);//0.4//5.0
    obj.set_use_embedded_materials(true); //colors set in the dae file
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    //flat_support for chessboard
    obj.setObjPath("package://chesslab_setup/models/flat_support/meshes/flat_support.dae");
    obj.setArucoID(0);
    obj.setKauthamName("flat_support");
    obj.setname("FLAT_SUPPORT");
    obj.set_frame_id("chess_frame");
    obj.setx(0);
    obj.sety(0);
    obj.setz(-chessboard_height/2.0); //reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/2.0));
    obj.setqw(cos(M_PI/2.0));
    obj.setscale_x(13.0); //65/5.0
    obj.setscale_y(14.0);//70.0/5.0
    obj.setscale_z(0.075);//0.4//5.0
    obj.set_use_embedded_materials(true); //colors set in the dae file
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);


    //Id made coincident with aruco marker id
    //Black pawns: 201 to 208
    //Black rooks: 209, 210
    //Black knights: 211, 212
    //Black kniwhts: 213, 214
    //Black queen and kings: 215, 216

    //pawn dimension 0.03x0.03x0.04
    //pawnB1
    obj.setObjPath("package://chesslab_setup/models/pawnB1/meshes/pawnB1.dae");
    obj.setArucoID(201);
    obj.setKauthamName("pawnB1");
    obj.setname("BLACK_PAWN_1");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB2
    obj.setObjPath("package://chesslab_setup/models/pawnB2/meshes/pawnB2.dae");
    obj.setArucoID(202);
    obj.setKauthamName("pawnB2");
    obj.setname("BLACK_PAWN_2");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB3
    obj.setObjPath("package://chesslab_setup/models/pawnB3/meshes/pawnB3.dae");
    obj.setArucoID(203);
    obj.setKauthamName("pawnB3");
    obj.setname("BLACK_PAWN_3");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB4
    obj.setObjPath("package://chesslab_setup/models/pawnB4/meshes/pawnB4.dae");
    obj.setArucoID(204);
    obj.setKauthamName("pawnB4");
    obj.setname("BLACK_PAWN_4");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB5
    obj.setObjPath("package://chesslab_setup/models/pawnB5/meshes/pawnB5.dae");
    obj.setArucoID(205);
    obj.setKauthamName("pawnB5");
    obj.setname("BLACK_PAWN_5");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(-0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB6
    obj.setObjPath("package://chesslab_setup/models/pawnB6/meshes/pawnB6.dae");
    obj.setArucoID(206);
    obj.setKauthamName("pawnB6");
    obj.setname("BLACK_PAWN_6");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(-0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB7
    obj.setObjPath("package://chesslab_setup/models/pawnB7/meshes/pawnB7.dae");
    obj.setArucoID(207);
    obj.setKauthamName("pawnB7");
    obj.setname("BLACK_PAWN_7");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(-0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnB8
    obj.setObjPath("package://chesslab_setup/models/pawnB8/meshes/pawnB8.dae");
    obj.setArucoID(208);
    obj.setKauthamName("pawnB8");
    obj.setname("BLACK_PAWN_8");
    obj.set_frame_id("chess_frame");
    obj.setx(0.125);
    obj.sety(-0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    //TowerB1
    obj.setObjPath("package://chesslab_setup/models/rookB1/meshes/rookB1.dae");
    obj.setArucoID(209);
    obj.setKauthamName("rookB1");
    obj.setname("BLACK_ROOK_1");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //TowerB2
    obj.setObjPath("package://chesslab_setup/models/rookB2/meshes/rookB2.dae");
    obj.setArucoID(210);
    obj.setKauthamName("rookB2");
    obj.setname("BLACK_ROOK_2");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(-0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //HorseB1
    obj.setObjPath("package://chesslab_setup/models/knightB1/meshes/knightB1.dae");
    obj.setArucoID(211);
    obj.setKauthamName("knightB1");
    obj.setname("BLACK_KNIGHT_1");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //HorseB2
    obj.setObjPath("package://chesslab_setup/models/knightB2/meshes/knightB2.dae");
    obj.setArucoID(212);
    obj.setKauthamName("knightB2");
    obj.setname("BLACK_KNIGHT_2");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(-0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //KnightB1
    obj.setObjPath("package://chesslab_setup/models/bishopB1/meshes/bishopB1.dae");
    obj.setArucoID(213);
    obj.setKauthamName("bishopB1");
    obj.setname("BLACK_BISHOP_1");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //KnightB2
    obj.setObjPath("package://chesslab_setup/models/bishopB2/meshes/bishopB2.dae");
    obj.setArucoID(214);
    obj.setKauthamName("bishopB2");
    obj.setname("BLACK_BISHOP_2");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(-0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //QueenB
    obj.setObjPath("package://chesslab_setup/models/queenB/meshes/queenB.dae");
    obj.setArucoID(215);
    obj.setKauthamName("queenB");
    obj.setname("BLACK_QUEEN");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //KingB
    obj.setObjPath("package://chesslab_setup/models/kingB/meshes/kingB.dae");
    obj.setArucoID(216);
    obj.setKauthamName("kingB");
    obj.setname("BLACK_KING");
    obj.set_frame_id("chess_frame");
    obj.setx(0.175);
    obj.sety(-0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);


    //White pawns: 301 to 308
    //White rooks: 309, 310
    //White knights: 311, 312
    //White kniwhts: 313, 314
    //White queen and kings: 315, 316

    //pawnW1
    obj.setObjPath("package://chesslab_setup/models/pawnW1/meshes/pawnW1.dae");
    obj.setArucoID(301);
    obj.setKauthamName("pawnW1");
    obj.setname("WHITE_PAWN_1");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW2
    obj.setObjPath("package://chesslab_setup/models/pawnW2/meshes/pawnW2.dae");
    obj.setArucoID(302);
    obj.setKauthamName("pawnW2");
    obj.setname("WHITE_PAWN_2");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW3
    obj.setObjPath("package://chesslab_setup/models/pawnW3/meshes/pawnW3.dae");
    obj.setArucoID(303);
    obj.setKauthamName("pawnW3");
    obj.setname("WHITE_PAWN_3");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW4
    obj.setObjPath("package://chesslab_setup/models/pawnW4/meshes/pawnW4.dae");
    obj.setArucoID(304);
    obj.setKauthamName("pawnW4");
    obj.setname("WHITE_PAWN_4");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW5
    obj.setObjPath("package://chesslab_setup/models/pawnW5/meshes/pawnW5.dae");
    obj.setArucoID(305);
    obj.setKauthamName("pawnW5");
    obj.setname("WHITE_PAWN_5");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW6
    obj.setObjPath("package://chesslab_setup/models/pawnW6/meshes/pawnW6.dae");
    obj.setArucoID(306);
    obj.setKauthamName("pawnW6");
    obj.setname("WHITE_PAWN_6");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW7
    obj.setObjPath("package://chesslab_setup/models/pawnW7/meshes/pawnW7.dae");
    obj.setArucoID(307);
    obj.setKauthamName("pawnW7");
    obj.setname("WHITE_PAWN_7");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //pawnW8
    obj.setObjPath("package://chesslab_setup/models/pawnW8/meshes/pawnW8.dae");
    obj.setArucoID(308);
    obj.setKauthamName("pawnW8");
    obj.setname("WHITE_PAWN_8");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.125);
    obj.sety(0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    //TowerB1
    obj.setObjPath("package://chesslab_setup/models/rookW1/meshes/rookW1.dae");
    obj.setArucoID(309);
    obj.setKauthamName("rookW1");
    obj.setname("WHITE_ROOK_1");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //TowerB2
    obj.setObjPath("package://chesslab_setup/models/rookW2/meshes/rookW2.dae");
    obj.setArucoID(310);
    obj.setKauthamName("rookW2");
    obj.setname("WHITE_ROOK_2");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //HorseB1
    obj.setObjPath("package://chesslab_setup/models/knightW1/meshes/knightW1.dae");
    obj.setArucoID(311);
    obj.setKauthamName("knightW1");
    obj.setname("WHITE_KNIGHT_1");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //HorseB2
    obj.setObjPath("package://chesslab_setup/models/knightW2/meshes/knightW2.dae");
    obj.setArucoID(312);
    obj.setKauthamName("knightW2");
    obj.setname("WHITE_KNIGHT_2");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //KnightB1
    obj.setObjPath("package://chesslab_setup/models/bishopW1/meshes/bishopW1.dae");
    obj.setArucoID(313);
    obj.setKauthamName("bishopW1");
    obj.setname("WHITE_BISHOP_1");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //KnightB2
    obj.setObjPath("package://chesslab_setup/models/bishopW2/meshes/bishopW2.dae");
    obj.setArucoID(314);
    obj.setKauthamName("bishopW2");
    obj.setname("WHITE_BISHOP_2");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //QueenW
    obj.setObjPath("package://chesslab_setup/models/queenW/meshes/queenW.dae");
    obj.setArucoID(315);
    obj.setKauthamName("queenW");
    obj.setname("WHITE_QUEEN");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);
    //KingW
    obj.setObjPath("package://chesslab_setup/models/kingW/meshes/kingW.dae");
    obj.setArucoID(316);
    obj.setKauthamName("kingW");
    obj.setname("WHITE_KING");
    obj.set_frame_id("chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(1.0);//3.0/5.0
    obj.setscale_y(1.0);//3.0/5.0
    obj.setscale_z(1.0);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);


    //set the index maps
    //kautham indices are automtically set according to the order in which they are written in the kautham problem xml file
    //rviz indices are set in the order they are loaded in the objects vector
    //rviz indices have been made coincident to kautham indices, although they could differ
    //the following maps relate the aruco markers with the rviz and kautham indices
    for(int i=0; i<objects.size();i++){
        //mymap.insert ( std::pair<char,int>('a',100) );
        aruco2kautham_map.insert(std::pair<unsigned int,std::string>(objects[i].getArucoID(), objects[i].getKauthamName()));
        kautham2aruco_map.insert(std::pair<std::string,unsigned int>(objects[i].getKauthamName(), objects[i].getArucoID()));
        aruco2rviz_map.insert(std::pair<unsigned int,unsigned int>(objects[i].getArucoID(), i));
        rviz2aruco_map.insert(std::pair<unsigned int,unsigned int>(i, objects[i].getArucoID()));
        aruco2ff_map.insert(std::pair<unsigned int,std::string>(objects[i].getArucoID(), objects[i].getname()));
        ff2aruco_map.insert(std::pair<std::string,unsigned int>(objects[i].getname(),objects[i].getArucoID()));
    }
}


//! Function that wraps the call to the kautham service that opens a problem
bool kauthamOpenProblem( string modelFolder, string problemFile )
{
    ros::NodeHandle n;
    ros::service::waitForService("/kautham_node/OpenProblem");

    kautham::OpenProblem kthopenproblem_srv;
    std::string model = modelFolder;
    ros::ServiceClient kthopenproblem_client = n.serviceClient<kautham::OpenProblem>("/kautham_node/OpenProblem");
    kthopenproblem_srv.request.problem = problemFile;
    kthopenproblem_srv.request.dir.resize(1);
    kthopenproblem_srv.request.dir[0] = model;
    kthopenproblem_client.call(kthopenproblem_srv);
    if (kthopenproblem_srv.response.response == true) {
        ROS_INFO( "Kautham Problem opened correctly" );
    } else {
        ROS_ERROR( "ERROR Opening Kautham Problem" );
        ROS_ERROR( "models folder: %s", kthopenproblem_srv.request.dir[0].c_str() );
        ROS_ERROR( "problem file: %s", kthopenproblem_srv.request.problem.c_str() );
        return false;
    }
    return true;
}




//! Function that wraps the call to the kautham service that checks for collisions
bool kauthamCheckCollision(std::vector<float> conf, std::string *collObjName, std::string *msg)
{
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/CheckCollision");
    ros::ServiceClient check_collision_obstacles_client = node.serviceClient<kautham::CheckCollision>("/kautham_node/CheckCollision");

    ROS_INFO("kauthamCheckCollision = (%f, %f, %f, %f, %f, %f, %f)", conf[0],conf[1],conf[2],conf[3],conf[4],conf[5],conf[6]);

    kautham::CheckCollision check_collision_obstacles_srv;
    check_collision_obstacles_srv.request.config = conf;

    check_collision_obstacles_client.call(check_collision_obstacles_srv);


    *collObjName = check_collision_obstacles_srv.response.collidedObs;
    *msg = check_collision_obstacles_srv.response.msg;

    if(!check_collision_obstacles_srv.response.collisionFree)
    {
        //std::cout<<" The current configuration is not collision free! "<<std::endl;
        return true;
    }
    else
    {
        //std::cout<<" The current configuration is collision free! "<<std::endl;
        return false;
    }
}



//! Function that wraps the call to the kautham service that moves an obstacle
bool kauthamSetObstaclePos(std::string obsname,  std::vector <float> pos)
{
    // activate the setObstaclePos service
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/SetObstaclePos");
    ros::ServiceClient set_obstacle_client = node.serviceClient<kautham::ObsPos>("/kautham_node/SetObstaclePos");

    kautham::ObsPos set_obstacle_srv;

    set_obstacle_srv.request.obsname = obsname;
    set_obstacle_srv.request.setPos = pos;

    //call kautham service to set the obstacle pos
    set_obstacle_client.call(set_obstacle_srv);

    // Evaluate the set obstacle service and perform rest of process
    if (set_obstacle_srv.response.response == false)
    {
        ROS_ERROR("SetObstaclePos service has not been performed. ");
        return false;
    }
    else
    {
        ROS_INFO("SetObstaclePos service has been performed !");
        return true;
    }
}


//! Function that wraps the call to the kautham service that attaches an obstacle to a robot link
bool kauthamAttachObs(int robotindex, int linkindex, std::string obsname)
{
    // activate the setObstaclePos service
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/AttachObstacle2RobotLink");
    ros::ServiceClient attach_client = node.serviceClient<kautham::AttachObstacle2RobotLink>("/kautham_node/AttachObstacle2RobotLink");

    kautham::AttachObstacle2RobotLink attach_srv;

    attach_srv.request.robot = robotindex;
    attach_srv.request.link = linkindex;
    attach_srv.request.obs = obsname;

    //call kautham service to set the robot pose
    attach_client.call(attach_srv);

    if (attach_srv.response.response == false)
    {
        ROS_ERROR("AttachObstacle2RobotLink service has not been performed. ");
        return false;
    }
    else
    {
        ROS_INFO("AttachObstacle2RobotLink service has been performed !");
        return true;
    }
}

//! Function that wraps the call to the kautham service that dettaches an obstacle from a robot
bool kauthamDettachObs(std::string obsname)
{
    // activate the setObstaclePos service
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/DetachObstacle");
    ros::ServiceClient dettach_client = node.serviceClient<kautham::DetachObstacle>("/kautham_node/DetachObstacle");

    kautham::DetachObstacle dettach_srv;

    dettach_srv.request.obsname = obsname;

    //call kautham service to set the robot pose
    dettach_client.call(dettach_srv);

    if (dettach_srv.response.response == false)
    {
        ROS_ERROR("DettachObstacle service has not been performed. ");
        return false;
    }
    else
    {
        ROS_INFO("DettachObstacle service has been performed !");
        return true;
    }
}



//! Service that sets the robot config in rviz and checks for collision using kautham
bool setrobconf(chesslab_setup::setrobconf::Request  &req,
         chesslab_setup::setrobconf::Response &res)
{
  //set conf to visualize in rviz
  conf[0] = req.conf[0];
  conf[1] = req.conf[1];
  conf[2] = req.conf[2];
  conf[3] = req.conf[3];
  conf[4] = req.conf[4];
  conf[5] = req.conf[5];
  conf[6] = req.conf[6];
  ROS_INFO("New robot-A configuration set to = (%f, %f, %f, %f, %f, %f, %f)", req.conf[0],req.conf[1],req.conf[2],req.conf[3],req.conf[4],req.conf[5],req.conf[6]);

  //move robot in kautham and collisioncheck
  //need to convert to normalized controls
  std::vector<float> controls(7);
  for(int i=0; i<6; i++){
      //<limit effort="54.0" lower="-3.141592654" upper="3.141592654" velocity="3.2"/>
      controls[i] = (conf[i]+3.141592654)/6.283185308;
  }
  //<limit effort="60" lower="0.0" upper="0.872664444444" velocity="1.91986177778"/>
  controls[6] = (conf[6]-0.0)/0.872664444444;

  ROS_INFO("New robot-A controls set to = (%f, %f, %f, %f, %f, %f, %f)", controls[0],controls[1],controls[2],controls[3],controls[4],controls[5],controls[6]);
  std::string collObjName;
  std::string msg;
  if(kauthamCheckCollision(controls, &collObjName, &msg)){
      res.obj = kautham2aruco_map.find(collObjName)->second;
      /*
      for(int i=0; i<objects.size(); i++) {
       ROS_DEBUG("object[%d].ID=%d collObj=%d",i,objects[i].getKauthamID(),collObj);
       if(objects[i].getKauthamID() == collObj){
           res.obj = objects[i].getArucoID();
           break;
       }
      }
      */

      //ROS_INFO("Collision with object %d",res.obj);
      res.msg = msg;
      res.incollision = true;
      ROS_INFO_STREAM(res.msg);
  }
  else{
      res.obj = -1;
      //ROS_INFO("Collision free configuration");
      res.incollision = false;
      res.msg = msg;
      ROS_INFO_STREAM(res.msg);
  }
  return true;
}


//! Service that sets the pose of an object in rviz and in kautham
bool setobjpose(chesslab_setup::setobjpose::Request  &req,
         chesslab_setup::setobjpose::Response &res)
{
  int i = aruco2rviz_map.find(req.objid)->second;
  objects[i].setx(req.p.position.x);
  objects[i].sety(req.p.position.y);
  objects[i].setz(req.p.position.z);
  objects[i].setqx(req.p.orientation.x);
  objects[i].setqy(req.p.orientation.y);
  objects[i].setqz(req.p.orientation.z);
  objects[i].setqw(req.p.orientation.w);

  /*
  int i;
  for(i=0; i<objects.size(); i++) {
   //ROS_DEBUG("object[%d].ID=%d req.objid=%d",i,objects[i].getArucoID(),req.objid);
   if(objects[i].getArucoID() == req.objid){
     objects[i].setx(req.p.position.x);
     objects[i].sety(req.p.position.y);
     objects[i].setz(req.p.position.z);
     objects[i].setqx(req.p.orientation.x);
     objects[i].setqy(req.p.orientation.y);
     objects[i].setqz(req.p.orientation.z);
     objects[i].setqw(req.p.orientation.w);
     break;
   }
  }
  */
  ROS_INFO("Object[%d] with id %d and kautham name \"%s\" set to pose = (%f, %f, %f, %f, %f, %f, %f)", i, req.objid, objects[i].getKauthamName().c_str(), req.p.position.x, req.p.position.y, req.p.position.z, req.p.orientation.x, req.p.orientation.y, req.p.orientation.z, req.p.orientation.w );

  //move object in kautham
  int collObj;
  std::string msg;
  std::string name = aruco2kautham_map.find(req.objid)->second; //objects[i].getKauthamID();
  std::vector <float> pos(7);
  pos[0] = req.p.position.x;
  pos[1] = req.p.position.y;
  pos[2] = req.p.position.z;
  pos[3] = req.p.orientation.x;
  pos[4] = req.p.orientation.y;
  pos[5] = req.p.orientation.z;
  pos[6] = req.p.orientation.w;

  kauthamSetObstaclePos(name, pos);
  return true;
}



//! Service that attachs an obstacle to the gripper_right pad.
bool attachobs2robot(chesslab_setup::attachobs2robot::Request  &req,
         chesslab_setup::attachobs2robot::Response &res)
{
  ROS_INFO("Attach obstacle with aruco mark %d to gripper right pad of robot %s", req.objarucoid, req.robotName.c_str());

  //Sets the transform that attaches the object to the robot gripper in rviz
  //it sets the frame of team_A__gripper_right_pad (or  team_B__gripper_right_pad) as the reference frame for the object
  //then computes the from this reference frame to the object reference frame

  //transform from chessboard to object
  int i = aruco2rviz_map.find(req.objarucoid)->second;
  tf2::Transform chess2obj;
  tf2::fromMsg(objects[i].getPose(), chess2obj);

  //transform from chessboard to robot gripper pad
  geometry_msgs::TransformStamped transformStamped;
  std::string rightpadframe = req.robotName+"_gripper_right_pad"; //team_A__gripper_right_pad or team_B_gripper_right_pad
  ROS_INFO("frame name = %s",rightpadframe.c_str());
  try{
      transformStamped = tfBuffer.lookupTransform("chess_frame", rightpadframe, ros::Time(0), ros::Duration(0.1));
  }
  catch (tf2::TransformException ex ){
    ROS_ERROR("%s",ex.what());
  }
  tf2::Stamped< tf2::Transform >  chess2pad;
  tf2::fromMsg(transformStamped, chess2pad);
  //std::cout<<"chess2pad!!!!!!! = "<<chess2pad.frame_id_<<std::endl;

  //transform from robot gripper pad to object
  tf2::Transform pad2chess = chess2pad.inverse();
  tf2::Transform pad2obj = pad2chess * chess2obj;

  //set the transform from right pad frame to oject
  objects[i].set_frame_id(rightpadframe);
  objects[i].setx(pad2obj.getOrigin().getX());
  objects[i].sety(pad2obj.getOrigin().getY());
  objects[i].setz(pad2obj.getOrigin().getZ());
  objects[i].setqx(pad2obj.getRotation().getX());
  objects[i].setqy(pad2obj.getRotation().getY());
  objects[i].setqz(pad2obj.getRotation().getZ());
  objects[i].setqw(pad2obj.getRotation().getW());

  /*
  std::cout<<"chess2pad = "<<std::endl;
  std::cout<<" x = "<<chess2pad.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<chess2pad.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<chess2pad.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<chess2pad.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<chess2pad.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<chess2pad.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<chess2pad.getRotation().getW()<< std::endl;
  std::cout<<"pad2chess = "<<std::endl;
  std::cout<<" x = "<<pad2chess.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<pad2chess.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<pad2chess.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<pad2chess.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<pad2chess.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<pad2chess.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<pad2chess.getRotation().getW()<< std::endl;
  std::cout<<"chess2obj = "<<std::endl;
  std::cout<<" x = "<<chess2obj.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<chess2obj.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<chess2obj.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<chess2obj.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<chess2obj.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<chess2obj.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<chess2obj.getRotation().getW()<< std::endl;

  std::cout<<"pad2obj = "<<std::endl;
  std::cout<<" x = "<<pad2obj.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<pad2obj.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<pad2obj.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<pad2obj.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<pad2obj.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<pad2obj.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<pad2obj.getRotation().getW()<< std::endl;
  */

  //attaches the object in kautham
  int robotindex;
  if(req.robotName=="team_A") robotindex = 0;
  else robotindex = 1;
  //std::cout<<"Robot index = "<<robotindex<<std::endl;
  int linkindex = 14; //this is the kautham index corresponding to the *gripper_right_pad*
  std::string obsname = aruco2kautham_map.find(req.objarucoid)->second; //objects[i].getKauthamID();
  kauthamAttachObs(robotindex, linkindex, obsname);

  //sets global flag on attached obstactes
  attached = req.objarucoid;

  std::cout<<"attached="<<attached<<std::endl;

  return true;
}



//! Service that dettaches an obstacle from a given robot.
bool dettachobs(chesslab_setup::dettachobs::Request  &req,
         chesslab_setup::dettachobs::Response &res)
{
  ROS_INFO("Dettach obstacle with aruco mark %d from robot %s", req.objarucoid, req.robotName.c_str());

  //Resets the transform that locates the object w.r.t. the chess frame

  //transform from robot gripper pad to object (the object is attached so the object pose is w.r.t the gripper pad frame)
  int i = aruco2rviz_map.find(req.objarucoid)->second;
  tf2::Transform pad2obj;
  tf2::fromMsg(objects[i].getPose(), pad2obj);

  //transform from chessboard to robot gripper pad
  geometry_msgs::TransformStamped transformStamped;
  std::string rightpadframe = req.robotName+"_gripper_right_pad"; //team_A__gripper_right_pad or team_B_gripper_right_pad
  ROS_INFO("frame name = %s",rightpadframe.c_str());
  try{
      transformStamped = tfBuffer.lookupTransform("chess_frame", rightpadframe, ros::Time(0), ros::Duration(0.1));
  }
  catch (tf2::TransformException ex ){
    ROS_ERROR("%s",ex.what());
  }
  tf2::Stamped< tf2::Transform >  chess2pad;
  tf2::fromMsg(transformStamped, chess2pad);

  //transform from chessboard to object
  tf2::Transform chess2obj = chess2pad * pad2obj;

  //set the transform from right pad frame to oject
  objects[i].set_frame_id("chess_frame");
  objects[i].setx(chess2obj.getOrigin().getX());
  objects[i].sety(chess2obj.getOrigin().getY());
  objects[i].setz(chess2obj.getOrigin().getZ());
  objects[i].setqx(chess2obj.getRotation().getX());
  objects[i].setqy(chess2obj.getRotation().getY());
  objects[i].setqz(chess2obj.getRotation().getZ());
  objects[i].setqw(chess2obj.getRotation().getW());

  //dettaches the object in kautham
  std::string obsname = aruco2kautham_map.find(req.objarucoid)->second;
  kauthamDettachObs(obsname);


  //resets global flag on attached obstactes
  std::cout<<"Current attached info"<<std::endl;
  std::cout<<"attached="<<attached<<std::endl;

  attached = -1;
  std::cout<<"resetting attached info"<<std::endl;
  std::cout<<"attached="<<attached<<std::endl;

  return true;
}




//! Function that wraps the call to the ur ik service
bool findIK(std::vector<std::vector<double>> &iksolution, geometry_msgs::Pose ur3pose)
{
    ros::NodeHandle node;
    ros::service::waitForService("/UR3IK");
    ros::ServiceClient ur3ik_client = node.serviceClient<ur3ik::UR3IK>("/UR3IK");
    ur3ik::UR3IK ur3ik_srv;

    ur3ik_srv.request.theta_ref.resize(6);
    ur3ik_srv.request.theta_min.resize(6);
    ur3ik_srv.request.theta_max.resize(6);
    for(int i=0; i<6; i++) {
        ur3ik_srv.request.theta_ref[i] = 0.0;
        ur3ik_srv.request.theta_min[i] = -M_PI;
        ur3ik_srv.request.theta_max[i] = M_PI;
    }
    ur3ik_srv.request.pose = ur3pose;


    ROS_INFO_STREAM("Robot Pose: [" <<
        ur3ik_srv.request.pose.position.x << ", " <<
        ur3ik_srv.request.pose.position.y << ", " <<
        ur3ik_srv.request.pose.position.z << ", " <<
        ur3ik_srv.request.pose.orientation.x << ", " <<
        ur3ik_srv.request.pose.orientation.y << ", " <<
        ur3ik_srv.request.pose.orientation.z << ", " <<
        ur3ik_srv.request.pose.orientation.w << "]");

    ur3ik_client.call(ur3ik_srv);

    std::stringstream sstr;
    if(ur3ik_srv.response.status)
    {
        iksolution.resize(ur3ik_srv.response.ik_solution.size());
        sstr<<"The computed ik is:"<<std::endl;
        for(int i=0; i<ur3ik_srv.response.ik_solution.size(); i++)
        {
            for(int j=0; j<6; j++)
                iksolution[i].push_back(ur3ik_srv.response.ik_solution[i].ik[j]);

            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << ur3ik_srv.response.ik_solution[i].ik[j] <<", ";
            }
            sstr << ur3ik_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
        return true;
    }
    else{
        ROS_INFO("Not able to compute the ik");
        return false;
    }
}


//! Service that calls the ik for the ur3 robot
bool inversekin(chesslab_setup::ik::Request  &req,
         chesslab_setup::ik::Response &res)
{
    std::vector<std::vector<double>> iksolution;
    if(findIK(iksolution, req.pose))
    {
        res.ik_solution.resize(iksolution.size());
        for(int i=0; i<iksolution.size(); i++)
            for(int j=0; j<6; j++)
                res.ik_solution[i].ik.push_back(iksolution[i][j]);
        res.status = true;
        return true;
    }
    else
    {
        res.status = false;
        return false;
    }
}


//! Function that wraps the call to the FF service that computes a plan
bool taskPlan(std::vector <std::string> &plan,
              std::vector< std::pair<std::string,std::string>> &init,
              std::vector< std::pair<std::string,std::string>> &goal,
              std::vector< std::string > &initfree,
              std::vector< std::string > &goalfree,
              std::string deadpiece="")
{
    ros::NodeHandle node;
    ros::service::waitForService("downward_service");
    ros::ServiceClient ff_client = node.serviceClient<downward_ros::Plan>("downward_service");
    downward_ros::Plan ff_srv;

    //Writing the problem file
    std::stringstream sstr_comment;
    std::stringstream sstr_intro;
    std::stringstream sstr_init;
    std::stringstream sstr_goal;
    std::stringstream sstr_problem;
    std::stringstream sstr_domain;

    sstr_comment << "; This is an automated generated file.\n\
    ; put in .gitignore not to be under version control\n";

    //Introductory part needed for all chess world problems
    sstr_intro << "(define (problem chessworld)\n\
        (:domain chessworld)\n\
        \n\
        (:objects \n\
            BLACK_PAWN_1 BLACK_PAWN_2 BLACK_PAWN_3 BLACK_PAWN_4 BLACK_PAWN_5 BLACK_PAWN_6 BLACK_PAWN_7 BLACK_PAWN_8 \n\
            WHITE_PAWN_1 WHITE_PAWN_2 WHITE_PAWN_3 WHITE_PAWN_4 WHITE_PAWN_5 WHITE_PAWN_6 WHITE_PAWN_7 WHITE_PAWN_8 \n\
            BLACK_BISHOP_1 BLACK_BISHOP_2 BLACK_KNIGHT_1 BLACK_KNIGHT_2 BLACK_ROOK_1 BLACK_ROOK_2 \n\
            WHITE_BISHOP_1 WHITE_BISHOP_2 WHITE_KNIGHT_1 WHITE_KNIGHT_2 WHITE_ROOK_1 WHITE_ROOK_2 \n\
            BLACK_KING BLACK_QUEEN \n\
            WHITE_KING WHITE_QUEEN \n\
            A1 A2 A3 A4 A5 A6 A7 A8  \n\
            B1 B2 B3 B4 B5 B6 B7 B8  \n\
            C1 C2 C3 C4 C5 C6 C7 C8  \n\
            D1 D2 D3 D4 D5 D6 D7 D8  \n\
            E1 E2 E3 E4 E5 E6 E7 E8  \n\
            F1 F2 F3 F4 F5 F6 F7 F8  \n\
            G1 G2 G3 G4 G5 G6 G7 G8  \n\
            H1 H2 H3 H4 H5 H6 H7 H8  \n\
            SAFE_REGION \n\
         )\n";

     //Setting the init state
     std::stringstream sstr_initmap;
     for(int i=0; i<init.size() ;i++)
     {
        sstr_initmap << "(on "+ init[i].first + " " +  init[i].second + ") \n";
     }
     std::stringstream sstr_initfree;
     for(int i=0; i<initfree.size() ;i++)
     {
        sstr_initfree << "(freecell "+ initfree[i] + ") \n";
     }
     sstr_init << "(:init \n\
        (arm-empty) \n\
        (freecell SAFE_REGION) \n" << sstr_initfree.str() << "\n" << sstr_initmap.str() << ")\n";
     //ROS_INFO_STREAM("Init set to " << sstr_init.str() << "\n freecells:" << sstr_initfree.str());


    //Setting the goal state
    std::stringstream sstr_goalmap;
    for(int i=0; i<goal.size() ;i++)
    {
        sstr_goalmap << "(on "+ goal[i].first + " " + goal[i].second +") \n";
    }
    std::stringstream sstr_goalfree;
    for(int i=0; i<goalfree.size() ;i++)
    {
       sstr_goalfree << "(freecell "+ goalfree[i] + ") \n";
    }

    sstr_goal << "(:goal \n\
        (and " << sstr_goalmap.str() << "\n" << sstr_goalfree.str();

    if(deadpiece!="")
        sstr_goal << " (deadpiece " << deadpiece << ")))\n";
    else
        sstr_goal <<  "))\n";

     //ROS_INFO_STREAM("Goal set to " << sstr_goal.str());

    //concatenate parts and close the problem
    sstr_problem << sstr_comment.str()+sstr_intro.str()+sstr_init.str()+sstr_goal.str()+")";
    
    //Writing it to a file
    std::string downward_ros_path = ros::package::getPath("downward_ros");
    std::string problemFileName = downward_ros_path + "/pddl/my_chess_world_problem";
    std::ofstream myproblemfile;
    myproblemfile.open (problemFileName, std::ofstream::out);
    myproblemfile << sstr_problem.str();
    myproblemfile.close();

    //Loading the request for the ff service: the problem file and the domain file    
    //evaluator and search can be left empty for FF since the downward server fills them with the ff info
    std::string domainFileName = downward_ros_path + "/pddl/chess_world.pddl";    
    std::ifstream mydomainfile(domainFileName);
    sstr_domain << mydomainfile.rdbuf();
    
    ff_srv.request.problem = sstr_problem.str();
    ff_srv.request.domain = sstr_domain.str();
    ff_srv.request.evaluator = "";
    ff_srv.request.search = "";

    

    ROS_INFO_STREAM("DomainFileName: " << domainFileName);
    ROS_INFO_STREAM("DomainFile: " << sstr_domain.str());
    ROS_INFO_STREAM("ProblemFileName: " << problemFileName);
    ROS_INFO_STREAM("ProblemFile: " << sstr_problem.str());

    //Calling the service
    ff_client.call(ff_srv);

    //printing the response
    std::stringstream sstr;
    if(ff_srv.response.response)
    {
        sstr<<"The computed ff plan is:"<<std::endl;
        for(int i=0; i<ff_srv.response.plan.size(); i++)
        {
            sstr << ff_srv.response.plan[i] <<std::endl;
            plan.push_back(ff_srv.response.plan[i]);
        }
        ROS_INFO_STREAM(sstr.str());
        return true;
    }
    else{
        ROS_INFO("Not able to compute ff plan");
        return false;
    }
}


//! Service that plans a movement using the FF task planner
bool planmovement(chesslab_setup::ffplan::Request  &req,
         chesslab_setup::ffplan::Response &res)
{
  ROS_INFO("Planning movements using FF");

  std::vector <std::string> ffplan;
  std::vector< std::pair<std::string,std::string> > init; //stores piecelabel and cell where it is located for the initial state
  std::vector< std::pair<std::string,std::string> > goal; //stores piecelabel and cell where it is located for the goal state
  std::string deadpiece;
  std::vector< std::string > initfree; //stores labels of free cells in init state
  std::vector< std::string > goalfree; //stores labels of free cells in goal state

  for(int i=0; i<req.init.objarucoid.size();i++)
    init.push_back( std::pair<std::string,std::string>(aruco2ff_map.find(req.init.objarucoid[i])->second, req.init.occupiedcells[i]));

  for(int i=0; i<req.goal.objarucoid.size();i++)
    goal.push_back( std::pair<std::string,std::string>(aruco2ff_map.find(req.goal.objarucoid[i])->second, req.goal.occupiedcells[i]));

  for(int i=0; i<req.init.freecells.size();i++)
    initfree.push_back( req.init.freecells[i]) ;

  for(int i=0; i<req.goal.freecells.size();i++)
    goalfree.push_back( req.goal.freecells[i]) ;

  if(req.killed) //when req.killed is not filled it evaluates to 0 and deadpiece remains a void string
    deadpiece = aruco2ff_map.find(req.killed)->second;


  for(int i=0; i<init.size();i++)
      std::cout<<"---- "<<init[i].first<<" "<<init[i].second<<std::endl;
  for(int i=0; i<goal.size();i++)
      std::cout<<"---- "<<goal[i].first<<" "<<goal[i].second<<std::endl;


  if(taskPlan(ffplan, init, goal, initfree, goalfree, deadpiece))
  {
      res.plan = ffplan;
      res.response = true;
  }
  else {
      res.response = false;
  }
  return true;
}

//! Service that sets the initial set-up
bool resetscene(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  ros::NodeHandle node;
  ROS_INFO("Setting the initial scene");

  //dettaches the object in kautham, if any was attached
  std::cout<<"attached="<<attached<<std::endl;
  if(attached!=-1)
  {
        kauthamDettachObs(aruco2kautham_map.find(attached)->second);
        attached = -1;
  }



  //Reset the pieces locations - this must be done after dettaching
  SetChessWorld();
  //Set pieces in Kautham
  std::string name;
  std::vector <float> pos(7);
  for(int i=0; i<objects.size(); i++)
  {
        name = objects[i].getKauthamName();
        pos[0] = objects[i].getPose().position.x;
        pos[1] = objects[i].getPose().position.y;
        pos[2] = objects[i].getPose().position.z;
        float qx = objects[i].getPose().orientation.x;
        float qy = objects[i].getPose().orientation.y;
        float qz = objects[i].getPose().orientation.z;
        float qw = objects[i].getPose().orientation.w;
        pos[3] = qx / sqrt(1-qw*qw);
        pos[4] = qy / sqrt(1-qw*qw);
        pos[5] = qz / sqrt(1-qw*qw);
        pos[6] = 2 * acos(qw);
        std::cout<<name<<": ["<<pos[0]<<", "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<", "<<pos[3]<<", "<<pos[4]<<", "<<pos[5]<<", "<<pos[6]<<std::endl;
        kauthamSetObstaclePos(name, pos);
  }


  //Reset the robots in rviz
  for(int i=0; i<7; i++)
    conf[i]  =  0.0;

  //Reset the robots in Kautham
  //need to convert to normalized controls
  std::vector<float> controls(7);
  for(int i=0; i<6; i++){
      //<limit effort="54.0" lower="-3.141592654" upper="3.141592654" velocity="3.2"/>
      controls[i] = (conf[i]+3.141592654)/6.283185308;
  }
  //<limit effort="60" lower="0.0" upper="0.872664444444" velocity="1.91986177778"/>
  controls[6] = (conf[6]-0.0)/0.872664444444;
  std::string collObjName;
  std::string msg;
  kauthamCheckCollision(controls, &collObjName, &msg);
  ROS_INFO_STREAM(msg);

  return true;
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "chesslab_setup_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    ROS_INFO("**** starting chesslab_setup node ****");

    ROS_INFO("**** starting kautham setup ****");
    string kautham_path = ros::package::getPath("kautham");
    string modelFolder = kautham_path + "/demos/models/";
    string problemFile = kautham_path + "/demos/OMPL_geo_demos/chess/OMPL_RRTconnect_chess_ur3_gripper_1_flat.xml";
    ROS_INFO("**** Loading %s ****",problemFile.c_str());

    kauthamOpenProblem(modelFolder, problemFile);

    ROS_INFO("**** starting rviz setup ****");
    SetChessWorld();
    // message declarations
    //teamA
    sensor_msgs::JointState joint_state_teamA_arm;
    joint_state_teamA_arm.name.resize(7);
    joint_state_teamA_arm.position.resize(7);
    joint_state_teamA_arm.position[0] = 0;
    joint_state_teamA_arm.position[1] = 0;
    joint_state_teamA_arm.position[2] = 0;
    joint_state_teamA_arm.position[3] = 0;
    joint_state_teamA_arm.position[4] = 0;
    joint_state_teamA_arm.position[5] = 0;
    joint_state_teamA_arm.position[6] = 0;

    //The node advertises the marker poses
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );

    //The node advertises the joint values for teamA robot
    ros::Publisher joint_pub_teamA_arm = n.advertise<sensor_msgs::JointState>("/team_A_arm/joint_states", 1);

    //The node advertises the joint values for teamA gripper
    ros::Publisher joint_pub_teamA_gripper = n.advertise<sensor_msgs::JointState>("/team_A_gripper/joint_states", 1);

    //The node provides a service to set a robot configuration
    ros::ServiceServer serviceRobConf = n.advertiseService("chesslab_setup/setrobconf", setrobconf);

    //The node provides a service to set the pose of a piece
    ros::ServiceServer serviceObjPose = n.advertiseService("chesslab_setup/setobjpose", setobjpose);

    //The node provides a service to attach a piece to the robot
    ros::ServiceServer serviceAttachObs2Rob = n.advertiseService("chesslab_setup/attachobs2robot", attachobs2robot);

    //The node provides a service to dettach a piece from the robot
    ros::ServiceServer serviceDettachObs = n.advertiseService("chesslab_setup/dettachobs", dettachobs);

    //The node provides a service to plan a chess movement
    ros::ServiceServer servicePlanmovement = n.advertiseService("chesslab_setup/planmovement", planmovement);

    //The node provides a service to compute the ur3ik
    ros::ServiceServer serviceIK = n.advertiseService("chesslab_setup/inversekin", inversekin);

    //The node provides a service to reset the scene to the inital configuration
    ros::ServiceServer serviceResetscene = n.advertiseService("chesslab_setup/resetscene", resetscene);


    tf2_ros::TransformListener tfListener(tfBuffer);
    std::vector <float> pos;
    int i;
    while (ros::ok())
    {
        i = 0;
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markers;


        //PIECES SETUP
        ROS_DEBUG("LOADING OBJECTS");
        ROS_DEBUG("%d OBJECTs",(int)objects.size());

        for(i=0; i<objects.size(); i++) {
            //marker.header.frame_id = "chess_frame";
            marker.header.frame_id = objects[i].get_frame_id();
            marker.header.stamp = ros::Time();
            marker.ns = "chess_pieces";
            marker.id = objects[i].getArucoID();
            marker.mesh_resource = objects[i].getObjPath();


            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose = objects[i].getPose();

            marker.scale.x = objects[i].getscale_x();
            marker.scale.y = objects[i].getscale_y();
            marker.scale.z = objects[i].getscale_z();
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = objects[i].getr();
            marker.color.g = objects[i].getg();
            marker.color.b = objects[i].getb();
            marker.mesh_use_embedded_materials = objects[i].get_use_embeded_materials();//if true the r,g,b peviously defined are overriden

            ROS_DEBUG("OBJECT: [%d]", marker.id);
            ROS_DEBUG("object path: %s", objects[i].getObjPath().c_str());
            ROS_DEBUG("X value is: [%f]", marker.pose.position.x);
            ROS_DEBUG("Y value is: [%f]", marker.pose.position.y);
            ROS_DEBUG("Z value is: [%f]", marker.pose.position.z);
            ROS_DEBUG("ORI X value is: [%f]", marker.pose.orientation.x);
            ROS_DEBUG("ORI Y value is: [%f]", marker.pose.orientation.y);
            ROS_DEBUG("ORI Z value is: [%f]", marker.pose.orientation.z);
            ROS_DEBUG("ORI W value is: [%f]", marker.pose.orientation.w);

            markers.markers.push_back(marker);
        }
        vis_pub.publish( markers );


        //ROBOT SETUP
        ROS_DEBUG("SETTING ROBOT");
        //update joint_state
        //teamA
        joint_state_teamA_arm.position[0] = conf[0];
        joint_state_teamA_arm.position[1] = conf[1];
        joint_state_teamA_arm.position[2] = conf[2];
        joint_state_teamA_arm.position[3] = conf[3];
        joint_state_teamA_arm.position[4] = conf[4];
        joint_state_teamA_arm.position[5] = conf[5];
        joint_state_teamA_arm.position[6] = conf[6];

        //teamA
        joint_state_teamA_arm.header.stamp = ros::Time::now();
        joint_state_teamA_arm.name[0] ="team_A_shoulder_pan_joint";
        joint_state_teamA_arm.name[1] ="team_A_shoulder_lift_joint";
        joint_state_teamA_arm.name[2] ="team_A_elbow_joint";
        joint_state_teamA_arm.name[3] ="team_A_wrist_1_joint";
        joint_state_teamA_arm.name[4] ="team_A_wrist_2_joint";
        joint_state_teamA_arm.name[5] ="team_A_wrist_3_joint";
        joint_state_teamA_arm.name[6] ="team_A_gripper_right_driver_joint";

        //send the joint state
        joint_pub_teamA_arm.publish(joint_state_teamA_arm);

        //ROS_INFO_STREAM("joints A: "<<joint_state_teamA_arm.position[0]<<"," \
                                    <<joint_state_teamA_arm.position[1]<<"," \
                                    <<joint_state_teamA_arm.position[2]<<"," \
                                    <<joint_state_teamA_arm.position[3]<<"," \
                                    <<joint_state_teamA_arm.position[4]<<"," \
                                    <<joint_state_teamA_arm.position[5]<<"," \
                                    <<joint_state_teamA_arm.position[6]);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

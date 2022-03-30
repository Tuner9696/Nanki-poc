#ifndef MOVE_CONTROL_HPP
#define MOVE_CONTROL_HPP

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>



/***************/
/* define定義  */
/***************/
#define SET_INITIAL_DATA  0
#define SET_FIRST_DATA    1
#define SET_NEXT_DATA     2
#define SET_COMPLETE_DATA 3
#define SET_ERROR_DATA    4
#define REGIST_STOP_DATA  5
#define CLEAR_STOP_DATA   6

#define RUN_KIND_AUTO   1 //自律走行
#define RUN_KIND_NOAUTO 2 //サーバ指示型走行
#define TOPIC_TRS_REQ "/trs_req" //移動/回転/停止指示
#define TOPIC_TRS_RES "/trs_res" //移動/回転/停止応答(リルート要求含む)
#define MAX_POINT_NUM 100 //サポートする最大ポイント数
#define MAX_ROBOT_NUM 100 //サポートする最大ロボット数


#define STOP_STATUS    0
#define RUNNING_STATUS 1

#define AUTOMATIC_RUNNING_LIMIT_TIME    200.0   //自律型走行限界所要時間係数
#define ROUTE_OUT_DISTANCE    0.2   //指示型走行到着判断角度係数

#define STRAIGHT_TIME_PROTECT_TIME     2.0   //指示型走行直進所要時間保護係数
#define ROTATIONAL_TIME_PROTECT_TIME   2.0   //指示型走行直進所要時間保護係数

#define ADJ_SPEED_ANGLE  5.0 //速度調整因子を発動させる目的角度までの差分角度(単位：°)
#define ADJ_SPEED_COEFF  0.3 //目的位置、角度に近づいた際の速度調整因子
#define ARRIVEL_DISTANCE 0.1 //指示型走行到着判断距離係数
#define ARRIVEL_ANGLE    1.0 //指示型走行到着判断角度係数

#define ARRIVAL    1
#define NO_ARRIVAL 0

#define NO_CNTL   0
#define STOP_CNTL 1
#define RESTART_CNTL 2



//経路情報
typedef struct
{
  int robot_id;
  int seq;
  int move_type;          //走行種別（自立型/指示型）
  int move_req;
  double pos_x;           //途中経路座標(x)
  double pos_y;           //途中経路座標(y)
  double pos_z;           //途中経路座標(z)
  double ori_p;           //途中経路角度(p
  double ori_r;           //途中経路角度(r)
  double ori_y;           //途中経路角度(y)
  double linear_x;                //直進速度(x)
  double linear_y;                //直進速度(y)
  double linear_z;                //直進速度(z)
  double angular_x;       //回転速度(x)
  double angular_y;       //回転速度(y)
  double angular_z;       //回転速度(z)
}RouteInfo;

typedef struct
{
    int route_num;                                     //経路No
    geometry_msgs::TransformStamped route_coordinates; //経路座標、向き
    geometry_msgs::Twist velocity;                     //直進速度、回転速度
} RobotMoveInfo;

//走行情報
typedef struct
{
        int robot_id;
        int move_type;
        int route_num;
        RobotMoveInfo robot_mofe_info[MAX_POINT_NUM];
}MoveInfo;

//内部状態
typedef struct
{
    int route_no;
    double pos_x;
    double pos_y;
    double pos_z;
    double ori_pitch;
    double ori_roll;
    double ori_yaw;
    double straight_speed;
    double rotational_speed;
} InternalRobotInfo;

//内部状態
typedef struct
{
    int route_no;
    double pos_x;
    double pos_y;
    double pos_z;
    double ori_x;
    double ori_y;
    double ori_z;
    double ori_w;
} tfRobotInfo;


typedef struct
{
    int action_status;               //動作状態
    int autonomous_move_status;     //自律走行状態
    int directive_move_status;      //指示走行状態
    double move_start_time;         //走行開始時間
    InternalRobotInfo last_route;   //前回経路情報
    InternalRobotInfo now_route;    //現在経路情報
    InternalRobotInfo stop_route;   //停止時経路情報
} InternalControlState;

//グローバル変数
extern RouteInfo g_route_info[MAX_ROBOT_NUM+1];
extern int g_move_ctrl_flag[MAX_ROBOT_NUM+1];
extern bool tfGet( std::string src, std::string dst, int ms, geometry_msgs::TransformStamped& pin );

//プロトタイプ宣言
void move_control_attach(ros::NodeHandle n);
void thread_move_ctrl(const int robotid, ros::NodeHandle n, MoveInfo **p_move_info);

#endif //MOVE_CONTROL_HPP

#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#define GAIN_CHASE -0.01
#define PNT_GOTO_ENEMY 6
#define PNT_START_CHASE 13

#define KP 0.003
#define KI 0.00
#define KD 0.00

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose {
  double x;
  double y;
  double yaw;
};

cv::Mat laserPoint(cv::Size(701,701),CV_8UC1,cv::Scalar(0));


double roll2, pitch2, yaw2;
double odomx,odomy;
double pc[360][2]={1.0};
int step=0;

MyPose way_point[] = {
    { 1.45,  0.0, 0.0},
    { 1.21,  0.0, 0.0},
    { 1.21,  0.0, M_PI/4},
    { 1.21 + 1.04 * sin(M_PI/4), 1.04 * cos(M_PI/4), M_PI/4},
    //{ 1.21 + 1.04 * sin(M_PI/4), 1.04 * cos(M_PI/4), M_PI/2},
    //{ 1.21 + 1.04 * sin(M_PI/4), 1.04 * cos(M_PI/4), -M_PI},
    //{ 1.21 + 1.04 * sin(M_PI/4) - 0.2, 1.04 * cos(M_PI/4), -M_PI},
    //{ 1.21 + 1.04 * sin(M_PI/4) - 0.2, 1.04 * cos(M_PI/4), 0.0},
    { 1.21 + 1.04 * sin(M_PI/4), 1.04 * cos(M_PI/4), 0.0},
    //{ 2.00 , 1.04 * cos(M_PI/4), 0.0},
    //{ 2.00 , 1.04 * cos(M_PI/4), -M_PI/2},
    //{ 2.00 , 1.04 * cos(M_PI/4)-0.2, -M_PI/2},
    //{ 2.00 , 1.04 * cos(M_PI/4), -M_PI/2},
    //{ 2.00 , 1.04 * cos(M_PI/4), 0.0},
    { 2.10 , 1.04 * cos(M_PI/4), 0.0},
    //{ 2.10 , 1.04 * cos(M_PI/4), 0.0},
    { 2.10 , 1.04 * cos(M_PI/4), -M_PI/4},

/*
    { 1.2,  0.0, 0.0},
    { 1.2,  0.0, M_PI/4},
    { 1.2 + 2.00 * sin(M_PI/4), 2.00 * cos(M_PI/4), M_PI/4},
    { 1.2 + 2.00 * sin(M_PI/4), 2.00 * cos(M_PI/4), -M_PI/4},
    { 1.2 + 2.00 * sin(M_PI/4) + 1.2 * sin(M_PI/4), 2.00 * cos(M_PI/4) - 1.2 * cos(M_PI/4), -M_PI/4},
	{ 1.3 + 2.00 * sin(M_PI/4) + 1.2 * sin(M_PI/4) + 0.3 * sin(M_PI/4), 2.00 * cos(M_PI/4) - 1.2 * cos(M_PI/4) - 0.3 * cos(M_PI/4), -M_PI/4},
	{ 1.3 + 2.00 * sin(M_PI/4) + 1.2 * sin(M_PI/4) + 0.3 * sin(M_PI/4) + ((2.00 * cos(M_PI/4) - 1.2 * cos(M_PI/4) - 0.3 * cos(M_PI/4)) / cos(M_PI/4)) * sin(M_PI/4), 0, -M_PI/4},
	{ 1.3 + 2.00 * sin(M_PI/4) + 1.2 * sin(M_PI/4) + 0.3 * sin(M_PI/4) + ((2.00 * cos(M_PI/4) - 1.2 * cos(M_PI/4) - 0.3 * cos(M_PI/4)) / cos(M_PI/4)) * sin(M_PI/4), 0, -M_PI},*/
	{999, 999, 999}};

static const std::string OPENCV_WINDOW = "Image window";
static const std::string LASER_WINDOW = "Image window2";

class RoboCtrl
{
private:
	enum EState
	{
		STATE_IDLE     = 0,
		STATE_WAYPOINT = 1,
		STATE_CHASE    = 2,
		STATE_STOP     = 3,
	};

public:
	RoboCtrl() : it_(node), ac("move_base", true)
	{
		ros::NodeHandle node;
		//購読するtopic名
		//odom_sub_ = node.subscribe("odom", 1, &RoboCtrl::odomCallback, this);
		image_sub_ = it_.subscribe("/blue_bot/camera/image_raw", 1, &RoboCtrl::imageCb, this);

		//配布するtopic名
		//twist_pub_ = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		twist_pub_ = node.advertise<geometry_msgs::Twist>("/blue_bot/cmd_vel", 1);

    //内部関数初期化
		m_frontspeed = 0.5;
		m_turnspeed = 0.0;
		m_diffPos = 0.0;

    // まず最初はウェイポイント
		m_state = STATE_WAYPOINT;

		initWaypoint();
		cv::namedWindow(OPENCV_WINDOW);
	}

	~RoboCtrl()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void initWaypoint()
	{
		// アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
		// この例では５秒間待つ(ブロックされる)
/*
		while(!ac.waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the move_base action server to come up");
		}
*/
		ROS_INFO("The server comes up");
		m_isSent = false;
		m_destPnt = 0;
	}

	void moveRobo()
	{
		//速度データ型宣言
		geometry_msgs::Twist twist;

		//checkWaypoint();
		sendWaypoint_laser(m_destPnt);

		/* STATE_STOPの場合 その場で停止 */
		if(m_state == STATE_STOP) {
			m_frontspeed = 0.0;
			m_turnspeed = 0.0;
			ROS_INFO("TEMP STOP!");
		}
/*
    	// STATE_CHASEの場合 P制御で追いかける
		if(m_state == STATE_CHASE) {
			ros::Duration tick = ros::Time::now() - m_timechasestr;
			double tickdbl = tick.toSec();
			if( tickdbl <= 8.0 ){
				m_frontspeed = 0.1;
				m_turnspeed = m_diffPos * GAIN_CHASE;
				ROS_INFO("CHASE PHASE(1) %f", tickdbl);
			} else if( tickdbl <= 16.0 ) {
				m_frontspeed = 0.0;
				m_turnspeed = m_diffPos * GAIN_CHASE;
				ROS_INFO("CHASE PHASE(2) %f", tickdbl);
			} else if( tickdbl > 16.0 )
				m_frontspeed = 0.0;
				m_turnspeed = -m_diffPos * GAIN_CHASE;
				ROS_INFO("CHASE PHASE(3) %f", tickdbl);
		}
*/
		if(m_state == STATE_CHASE) {

			ROS_INFO("追跡中");
			ros::Time dif_time1;
      			static ros::Time dif_time2 = m_timechasestr;
			static double dif_err0 = 0;
			static double dif_err1 = 0;
			static double m_int = 0;
			static double m_dif = 0;
			static double tickdbl = 0;
			//dif_time2 = m_timechasestr;

				dif_time1 = dif_time2;
				dif_time2 = ros::Time::now();
				ros::Duration tick = dif_time2 - dif_time1;
				tickdbl = tick.toSec();
				ROS_INFO("時間差%lf",tickdbl);
				dif_err0 = dif_err1;
				dif_err1 = m_diffPos;
				m_int += ((dif_err1 + dif_err0)/2) * tickdbl;
				m_dif = (dif_err1 - dif_err0)/tickdbl;
				m_frontspeed = 0.0;
				m_turnspeed = (m_diffPos * KP + m_int * KI + m_dif * KD)*(-1);
				ROS_INFO("偏差%lf",m_diffPos);
				ROS_INFO("回転速度%lf",m_turnspeed);
				ros::Duration(0.1).sleep();
			if (m_diffPos == 0){
				m_state = STATE_WAYPOINT;
				ROS_INFO("制御終了");
			}

			

		}

    	// ウェイポイント終わったらSTATE_IDLEにしてその場で回る
		if(m_state == STATE_IDLE) {
			m_frontspeed = -0.1;
			m_turnspeed = 0.0;
			ROS_INFO("今の状態アイドル");
			ROS_INFO("diff %f", m_diffPos);
		}

		//ROS_INFO("NOW %d", m_state);
		//ROS_INFO("diff %f", m_diffPos);

		//ROS速度データに内部関数値を代入
		//if( m_state == STATE_CHASE || m_state == STATE_IDLE || m_state == STATE_STOP){
			twist.linear.x = m_frontspeed;
			twist.angular.z = m_turnspeed;
			twist_pub_.publish(twist);
		//}
	}
/*
	void checkWaypoint()
	{
		// ウェイポイント送信済みか？
		if( m_isSent ) {
		// ウェイポイント送信済みなら到着結果の確認をする
		// 追跡モード中はm_isSentはfalseなので確認しない
			bool isSucceeded = ac.waitForResult(ros::Duration(0.5));
			// 結果を見て、成功ならSucceeded、失敗ならFailedと表示
			actionlib::SimpleClientGoalState state = ac.getState();

			// 到着済みか?
			if( isSucceeded ) {
			// 到着済みなら次の点を送信する
				m_isSent = false;
				ROS_INFO("WP Reached: No.%d (%s)",m_destPnt+1, state.toString().c_str());
				ros::Duration(1.0).sleep();
				sendWaypoint(++m_destPnt);
			}else {
				// 到着してないなら何もしない
				ROS_INFO("WP not reached: No.%d (%s)",m_destPnt+1, state.toString().c_str());
			}
		} else {
		// ウェイポイント送信済みでない場合
			if( m_state == STATE_WAYPOINT ){
			// STATE_WAYPOINTのときウェイポイント送信(初回時やSTATE_CHASEから復帰時)
			// STATE_CHASE, STATE_IDLEでは何もしない)
				sendWaypoint(m_destPnt);
			}
		}
	}
*/
	void sendWaypoint_laser(int n_ptr){
		int state_way=0;
		double x_d,y_d,phi,odomt,t_tar;
		double K_phi=0.4;
		double K_r=0.4;
		// STATE_WAYPOINTの場合のみ呼ばれる
		// ウェイポイントの更新 or 初回時 or STATE_CHASEからの復帰
		//move_base_msgs::MoveBaseGoal goal;
		// map(地図)座標系
		//goal.target_pose.header.frame_id = "map";
		// 現在時刻
		//goal.target_pose.header.stamp = ros::Time::now();
		//goal.target_pose.pose.position.x =  way_point[n_ptr].x;
		//goal.target_pose.pose.position.y =  way_point[n_ptr].y;
		//goal.target_pose.pose.position.x =  way_point[n_ptr].x;
		//goal.target_pose.pose.position.y =  way_point[n_ptr].y;
		// 最終ウェイポイントに達したらSTATE_IDLEに遷移して抜ける。
		

		if (way_point[n_ptr].x/*goal.target_pose.pose.position.x*/ == 999) {
			m_state = STATE_IDLE;
			return;
		}

		phi=asin((way_point[n_ptr].y-odomy)/sqrt((odomx-way_point[n_ptr].x)*(odomx-way_point[n_ptr].x)+(odomy-way_point[n_ptr].y)*(odomy-way_point[n_ptr].y)));
		//r=sqrt((odomx-way_point[n_ptr].x)*(odomx-way_point[n_ptr].x)+(odomy-way_point[n_ptr].y)*(odomy-way_point[n_ptr].y));
		//odomt=odomx*cos(phi)+odomy*sin(phi);
		t_tar=(way_point[n_ptr].x-odomx)*cos(phi)+(way_point[n_ptr].y-odomy)*sin(phi);
		ROS_INFO("phi-yaw2:%f way_point.yaw-yaw2:%f", phi-yaw2,way_point[n_ptr].yaw-yaw2);
		ROS_INFO("phi:%f t_tar:%f odomx:%f odomy:%f yaw2:%f",phi,t_tar,odomx,odomy,yaw2);

		if(((phi-yaw2)>1*M_PI/180 || (phi-yaw2)<-1*M_PI/180) && (t_tar>0.01 || t_tar<-0.01)){//step1
			m_frontspeed=0;
			m_turnspeed=K_phi*(phi-yaw2);	
			if(step!=1){
				//m_frontspeed=0;
				//m_turnspeed=0;
			}
			ROS_INFO("step1!!");
			step=1;
		}
		else if(((phi-yaw2)<1*M_PI/180&&(phi-yaw2)>-1*M_PI/180) && (t_tar>0.01 || t_tar<-0.01)){//step2
			m_frontspeed=K_r*t_tar;
			m_turnspeed=0;	
			ROS_INFO("step2!!");
			if(step!=2){
				//m_frontspeed=0;
				//m_turnspeed=0;
			}
			step=2;
		}
		else if(((way_point[n_ptr].yaw-yaw2)>1*M_PI/180||(way_point[n_ptr].yaw-yaw2)<-1*M_PI/180) && (t_tar<0.01 && t_tar>-0.01) ){//step3
			m_frontspeed=0;
			m_turnspeed=K_phi*(way_point[n_ptr].yaw-yaw2);	
			ROS_INFO("step3!!");
			if(step!=3){
				//m_frontspeed=0;
				//m_turnspeed=0;
			}
			step=3;
		}
		else if(((way_point[n_ptr].yaw-yaw2)<1*M_PI/180&&(way_point[n_ptr].yaw-yaw2)>-1*M_PI/180) && (t_tar<0.01 && t_tar>-0.01)){//step4
			m_frontspeed=0;
			m_turnspeed=0;
			m_destPnt++;	
			ROS_INFO("step4!!");
			step=4;
		}

		//goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[n_ptr].yaw);

		ROS_INFO("Sending goal: No.%d", n_ptr+1);
		ROS_INFO("X=%f Y=%f th=%f", way_point[n_ptr].x, way_point[n_ptr].y, way_point[n_ptr].yaw);

		// サーバーにgoalを送信
		//ac.sendGoal(goal);

		m_isSent = true;
	}
/*
	void sendWaypoint(int n_ptr){
		// STATE_WAYPOINTの場合のみ呼ばれる
		// ウェイポイントの更新 or 初回時 or STATE_CHASEからの復帰
		move_base_msgs::MoveBaseGoal goal;
		// map(地図)座標系
		goal.target_pose.header.frame_id = "map";
		// 現在時刻
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x =  way_point[n_ptr].x;
		goal.target_pose.pose.position.y =  way_point[n_ptr].y;

		// 最終ウェイポイントに達したらSTATE_IDLEに遷移して抜ける。
		if (goal.target_pose.pose.position.x == 999) {
			m_state = STATE_IDLE;
			return;
		}

		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[n_ptr].yaw);

		ROS_INFO("Sending goal: No.%d", n_ptr+1);
		ROS_INFO("X=%f Y=%f th=%f", way_point[n_ptr].x, way_point[n_ptr].y, way_point[n_ptr].yaw);

		// サーバーにgoalを送信
		ac.sendGoal(goal);

		m_isSent = true;
	}
*/


	void cancelWayPoint()
	{
		// ウェイポイントのキャンセル。STATE_CHASEへの移行時に呼ばれる
		ac.cancelAllGoals();
		ROS_INFO("WAYPOINT CANCELED. START CHASING!!");

		// STATE_WAYPOINT復帰時にウェイポイントを送り直すようにしておく
		m_isSent = false;
	}

	void odomCallback(const nav_msgs::Odometry &odom)
	{
		return;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{

		const int centerpnt = 320;
		const int range = 30;
		static EState laststate = m_state;

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::cvtColor(cv_ptr->image,hsv,CV_BGR2HSV);
		cv::inRange(hsv, cv::Scalar(60-range, 100, 100), cv::Scalar(60+range, 255, 255), mask); // 色検出でマスク画像の作成
		//cv::bitwise_and(cv_ptr->image,mask,image);

		cv::Moments mu = cv::moments( mask, false );
		cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

		double area = mu.m00;
		int x = mu.m10/mu.m00;
		int y = mu.m01/mu.m00;
		ROS_INFO("AREA = %f", area);


		/* 敵が居ぬ間に後側に回りこむ */
		if( (x >= 0) && (x <= 640) && (area > 2000) && (m_destPnt == PNT_GOTO_ENEMY)){
			/* 敵が居たらストップ */
			if( m_state == STATE_WAYPOINT){
				laststate = m_state;
				cancelWayPoint();
			}
			m_state = STATE_STOP;

		}else if( (x >= 0) && (x <= 640) && (area > 20000) && (m_destPnt > PNT_START_CHASE)){
			// 敵が見つかったら追跡する
			m_diffPos = x - centerpnt;

			// STATE_CHASEに入る前の状態を保存
			switch( m_state ){
				case STATE_WAYPOINT:
					laststate = m_state;	///
					cancelWayPoint();
				case STATE_IDLE:
					laststate = m_state;
					m_timechasestr = ros::Time::now();
					break;
				default:
					break;
			}
			// STATE_CHASEに遷移
			m_state = STATE_CHASE;

		} else {
			// 敵を見失う or そもそも敵を見つけていない場合
			m_diffPos = 0;
			// STATE_CHASEに入る前の状態を戻す
			if( m_state == STATE_CHASE ){
				m_state = laststate;
			}
		}

		//ROS_INFO("obj x=%d y=%d",x,y);

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, mask);
		cv::waitKey(3);

	}

	private:
		ros::NodeHandle node;
		ros::Subscriber odom_sub_;
		ros::Subscriber bumper_sub_;
		ros::Subscriber Laser_sub_;
		ros::Publisher twist_pub_;

		//ROS時間
		ros::Time push_time_;
		//ROS変化秒
		ros::Duration under_time_;

		cv::Mat hsv;
		cv::Mat mask;
		cv::Mat image;

		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;

		EState m_state;

		// ロボット制御用
		double m_diffPos;
		double m_frontspeed;
		double m_turnspeed;
    ros::Time m_timechasestr;


		// ウェイポイント制御用
		int m_destPnt;
		bool m_isSent;
		MoveBaseClient ac;
};

	void odomCallback2(const nav_msgs::Odometry &odom)
	{
		odomx = odom.pose.pose.position.x;
		odomy = odom.pose.pose.position.y;
		//double odomyaw = odom.pose.pose.orientation;
/*
tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
tf::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
pose2d.theta = yaw;
*/
tf::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);
tf::Matrix3x3 m(q);

m.getRPY(roll2, pitch2, yaw2);
                //ROS_INFO("odom x y yaw= %f %f %f", odomx,odomy,yaw2);
		//return;
	}

void scancallback(const sensor_msgs::LaserScan::ConstPtr& input) {
	// Solve all of perception here...
	//cv::Mat image_depth = cv_bridge::toCvCopy(msg)->image;
	//pc[0][0]=double(msg->range_max);
	sensor_msgs::LaserScan output;

	//sensor_msgs::MultiEchoLaserScan muls;
	for (int i = 0; i < input->ranges.size(); ++i) {
		const float &range = input->ranges[i];
		//ROS_INFO("ranges: [%f]",range);
		//sensor_msgs::LaserEcho echo;
		//echo.echoes.push_back(range);
		//muls.ranges.push_back(echo);
		pc[i][0]=(double(range)*cos(i*M_PI/180+yaw2)+odomx)*cos(-M_PI/4)-(double(range)*sin(i*M_PI/180+yaw2)+odomy)*sin(-M_PI);
		pc[i][1]=(double(range)*sin(i*M_PI/180+yaw2)+odomx)*sin(-M_PI/4)+(double(range)*sin(i*M_PI/180+yaw2)+odomy)*cos(-M_PI);
	}
}

int main(int argc, char **argv)
{
	//ROSのノード初期化
	ros::init(argc, argv, "robo_ctrl");
	RoboCtrl robo_ctrl;
	ros::Rate r(20);
	ros::NodeHandle n;
        ros::Subscriber odom_sub_loop;
	ros::Subscriber scan_sub;
	for(int i=0;i<359;i++){
		laserPoint.data[i] = 1;
	}
	while (ros::ok())
	{
		robo_ctrl.moveRobo();
		//購読するtopic名
		odom_sub_loop = n.subscribe("/blue_bot/odom", 1000, odomCallback2);
		scan_sub = n.subscribe<sensor_msgs::LaserScan>("/blue_bot/scan", 1000, scancallback);
                //ROS_INFO("odom x y yaw= %f %f %f", odomx,odomy,yaw2);
		int point_num=round(3.1400001049*2/0.0174930356443);
/*
		for(int i=0;i<point_num;i++){
                	ROS_INFO("x::: %f",pc[i][0]);
			ROS_INFO("y::: %f",pc[i][1]);
		}
*/
/*
	for(int k=0;k<359;k++){
		for(int i=0;i<701;i++){
		    for(int j=0;j<701;j++){
			if(i*0.005<=pc[k][0]&&pc[k][0]<(i+1)*0.005 && j*0.005<=pc[k][1]&&pc[k][1]<(j+1)*0.005){
				laserPoint.data[i*701+j] = 255;
			}
		    }
		}
	 }
*/
		//laserPoint.data[245700] = 255;
		//cv::imshow(LASER_WINDOW, laserPoint);
		ros::spinOnce();
		r.sleep();
	}
}

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "custom_msgs/gnssSample.h"
#include "sherpa_msgs/GeoPoint.h"
#include "std_msgs/Float64.h"
#include "math.h"
#include "donkey_rover/Rover_Track_Speed.h"
#include <wgs84_ned_lib/wgs84_ned_lib.h> 

// TODO here include the message definitions

bool is_GPS_present = false;
bool HOME = false;
float GPS_msg_count = 0;
bool GPS_first_msg = false;
const float pi = 3.1415926535;
float it_num = 1;
float VR = 0;
float VL = 0;
float V_xB = 0;
bool flag_init = true;
float cosPhi,sinPhi,cosLambda,sinLambda,t;

class PosEstimNodeClass
{
	public:
			
		PosEstimNodeClass(ros::NodeHandle& node)
		{
			n_=node;

			//subscribers
			subFromImu_ = n_.subscribe("/mti/sensor/imu", 10, &PosEstimNodeClass::readImuMessage,this);
			subFromGPS_ = n_.subscribe("/mti/sensor/gnssPvt", 10, &PosEstimNodeClass::readGPSMessage,this);
			// subFromOdo_ = n_.subscribe("/odom", 10, &PosEstimNodeClass::readOdoMessage,this);
			subFromTrackSpeed_ = n_.subscribe("RoverTrackSpeed", 10, &PosEstimNodeClass::readTrackSpeedMessage,this);
			subFromHome_= n_.subscribe("/home", 10, &PosEstimNodeClass::readHomeMessage,this);
			
			// publishers
			pubEstimatedPosition_= n_.advertise<position_estimator::estimated_position>("/estimated_position", 10);	
		}

		void readImuMessage(const sensor_msgs::Imu::ConstPtr& msg)
		{
			inputImu_.orientation.w = msg->orientation.w; 
			inputImu_.orientation.x = msg->orientation.x; 
			inputImu_.orientation.y = msg->orientation.y; 
			inputImu_.orientation.z = msg->orientation.z; 
			//ROS_INFO("NEW IMU MSG"); // ONLY FOR DEBUG
			//ROS_INFO("q1, q2, q3, q4, %.5f, %.5f, %.5f, %.5f",inputImu_.orientation.w, inputImu_.orientation.x, inputImu_.orientation.y,inputImu_.orientation.z); // ONLY FOR DEBUG
		}

		void readGPSMessage(const custom_msgs::gnssSample::ConstPtr& msg) 				
		{	
			/*  # This is a message to hold data a GNSS unit
				# Supported for MTi Devices with FW 1.4 and above.

				Header header

				float64 itow
				float64 fix

				float64 latitude
				float64 longitude
				float64 hEll
				float64 hMsl

				# ENU velocity
				geometry_msgs/Vector3 vel // float64

				float64 hAcc
				float64 vAcc
				float64 sAcc

				float64 pDop
				float64 hDop
				float64 vDop

				float64 numSat
				float64 heading
				float64 headingAcc
			*/
			inputGPS_.heading = msg->heading; 
			inputGPS_.hMsl = msg->hMsl; 
			inputGPS_.hEll = msg->hEll; 
			inputGPS_.latitude = msg->latitude;
			inputGPS_.longitude = msg->longitude;
			inputGPS_.vel.x = msg->vel.x; //East
			inputGPS_.vel.y = msg->vel.y; //North
			inputGPS_.vel.z = msg->vel.z; //Up
		
			ROS_INFO("NEW GPS MSG, LAT, LON %.7f, %.7f",inputGPS_.latitude, inputGPS_.longitude); // ONLY FOR DEBUG
		
			is_GPS_present = true;
			GPS_msg_count = GPS_msg_count+1;
			ROS_INFO("GPS_msg_count %.0f",GPS_msg_count); // ONLY FOR DEBUG
			if (GPS_msg_count == 1)
			{
				GPS_first_msg = true;
			}
			else if (GPS_msg_count > 1)
			{
				GPS_msg_count = 2;
				GPS_first_msg = false;
			}
		}

		void readTrackSpeedMessage(const donkey_rover::Rover_Track_Speed::ConstPtr& msg) 
		{
			/*
			float64 Front_Right_Track_Speed
			float64 Front_Left_Track_Speed
			float64 Rear_Right_Track_Speed
			float64 Rear_Left_Track_Speed
			*/
			
			inputTrackSpeed_.Front_Right_Track_Speed = msg->Front_Right_Track_Speed;
			inputTrackSpeed_.Front_Left_Track_Speed = msg->Front_Left_Track_Speed;
			inputTrackSpeed_.Rear_Right_Track_Speed = msg->Rear_Right_Track_Speed;
			inputTrackSpeed_.Rear_Left_Track_Speed = msg->Rear_Left_Track_Speed;
			
			VR = (inputTrackSpeed_.Front_Right_Track_Speed+inputTrackSpeed_.Rear_Right_Track_Speed)/2.0;
			VL = (inputTrackSpeed_.Front_Left_Track_Speed+inputTrackSpeed_.Rear_Left_Track_Speed)/2.0;
			V_xB = (VR+VL)/2.0; // rover x-body speed in m/s
			
			//ROS_INFO("NEW TRACK SPEED MSG"); // ONLY FOR DEBUG
			//ROS_INFO("VR, VL, %.7f, %.7f",VR,VL); // ONLY FOR DEBUG
		}
		
		void readHomeMessage(const sherpa_msgs::GeoPoint::ConstPtr& msg) // TODO
		{
			/*inputHome_.home =  msg-> home;
			if (inputHome_.home == true)
			{
				HOME = true;
			}*/
			P_HOME_GPS = *msg;
			
			is_home_set = true;
		}
		
		class v_NED
		{ // this class defines a NED vector
			public:
			float N;
			float E;
			float D;
			v_NED()
			{
				N = 0.0; E = 0.0; D = 0.0;
			}
		};
		//v_NED  P_ACTUAL_NED;
		
		class v_ECEF
		{ // this class defines a NED vector
			public:
			float X;
			float Y;
			float Z;
		};
		v_NED  d_hat_P_NED;
		v_NED  P_NED_GPS;
		v_ECEF P_ACTUAL_ECEF;
		v_NED  P_ACTUAL_NED;
        v_ECEF DP_ECEF;
		v_ECEF P_HOME_ECEF;
		v_NED  hat_P_NED;
/*		class v_WGS84
		{// this class defins a WGS84 vector
			public:
			float64 lat
			float64 lon
			float64 alt // hEll			
		}*/  
		
		v_ECEF GPS2ECEF(custom_msgs::gnssSample P_ACTUAL_GPS)
		{
			v_ECEF P_ACTUAL_ECEF;
            float cosPhi,sinPhi,cosLambda,sinLambda,e,N;

			float a = 6378137.0;
			float b = 6356752.3142;
			float pi = 3.1415926535;
			
			cosPhi = cos(P_ACTUAL_GPS.latitude/180.0*pi);
			sinPhi = sin(P_ACTUAL_GPS.latitude/180.0*pi);
			cosLambda = cos(P_ACTUAL_GPS.longitude/180.0*pi);
			sinLambda = sin(P_ACTUAL_GPS.longitude/180.0*pi);
			//ROS_INFO("c(Phi), s(Phi), c(Lam), s(Lam) %.3f, %.3f, %.3f, %.3f",cosPhi,sinPhi,cosLambda,sinLambda);
			e = sqrt(1.0-(pow(b,2))/(pow(a,2)));
			N = a/sqrt(1.0-pow(e,2)*pow(sinPhi,2));
			//ROS_INFO("e, N %.3f, %.3f",e,N);			
			P_ACTUAL_ECEF.X = (N+P_ACTUAL_GPS.altitude)*cosPhi*cosLambda;
			P_ACTUAL_ECEF.Y = (N+P_ACTUAL_GPS.altitude)*cosPhi*sinLambda;
			P_ACTUAL_ECEF.Z = (((pow(b,2)/pow(a,2))*N+P_ACTUAL_GPS.altitude))*sinPhi;			
			//ROS_INFO("P_ECEF X, Y, Z %f, %f, %f",P_ACTUAL_ECEF.X, P_ACTUAL_ECEF.Y, P_ACTUAL_ECEF.Z);
			return P_ACTUAL_ECEF;
		}
		
		/*v_NED GPS2NED(custom_msgs::gnssSample P_ACTUAL_GPS, v_ECEF P_HOME_ECEF)
		{
			v_NED  P_ACTUAL_NED;
			v_ECEF P_ACTUAL_ECEF;
			// v_ECEF P_HOME_ECEF;
            v_ECEF DP_ECEF;

            float cosPhi,sinPhi,cosLambda,sinLambda,t;
			
			// P_HOME_ECEF = GPS2ECEF(P_HOME_GPS);
			P_ACTUAL_ECEF = GPS2ECEF(P_ACTUAL_GPS);
			// ROS_INFO("P_HOME_ECEF, %.3f,%.3f,%.3f",P_HOME_ECEF.X,P_HOME_ECEF.Y,P_HOME_ECEF.Z); // ONLY FOR DEBUG
			ROS_INFO("P_ACTUAL_ECEF, %.3f,%.3f,%.3f",P_ACTUAL_ECEF.X,P_ACTUAL_ECEF.Y,P_ACTUAL_ECEF.Z); // ONLY FOR DEBUG
			
			DP_ECEF.X = P_ACTUAL_ECEF.X - P_HOME_ECEF.X;
            DP_ECEF.Y = P_ACTUAL_ECEF.Y - P_HOME_ECEF.Y; 
            DP_ECEF.Z = P_ACTUAL_ECEF.Z - P_HOME_ECEF.Z;
			
			cosPhi = cos(P_HOME_GPS.latitude/180.0*pi);
			sinPhi = sin(P_HOME_GPS.latitude/180.0*pi);
			cosLambda = cos(P_HOME_GPS.longitude/180.0*pi);
			sinLambda = sin(P_HOME_GPS.longitude/180.0*pi);
			t =  cosLambda* DP_ECEF.X + sinLambda* DP_ECEF.Y;
			
			P_ACTUAL_NED.N = -sinPhi * t + cosPhi * DP_ECEF.Z;
			P_ACTUAL_NED.E = -sinLambda * DP_ECEF.X + cosLambda * DP_ECEF.Y;
			P_ACTUAL_NED.D = -(cosPhi * t + sinPhi * DP_ECEF.Z);
			
			return P_ACTUAL_NED;
		}*/
		
		void PositionEstimation_Handle()
		{
			
			if (flag_init == true)
			{
				hat_P_NED.N = 0.0;
				hat_P_NED.E = 0.0;
				hat_P_NED.D = 0.0;	
				flag_init = false;
				ROS_ERROR("hat_P_NED(0) %f,%f,%f ",hat_P_NED.N,hat_P_NED.E,hat_P_NED.D);
			}
			
            float q1,q2,q3,q0;

			// K = filter gain
			float KN = 0.1;
			float KE = 0.1;
			float KD = 0.1;
			
			// VR, VL = right and left wheel speed
			// V_xB = (inputTrackSpeed_.R+inputTrackSpeed_.L)/2.0; // rover x-body speed
			
			//XSENSE FORMAT
			//R_B2ENU	=	see MTi_usermanual.pdf page 40 	
			q0 = inputImu_.orientation.w; // scalar component					   
			q1 = inputImu_.orientation.x;
			q2 = inputImu_.orientation.y;
			q3 = inputImu_.orientation.z;
			
			d_hat_P_NED.N = (2*pow(q0,2)+2*pow(q2,2)-1)*V_xB;
			d_hat_P_NED.E = 2*(q1*q2-q3*q0)*V_xB;
			d_hat_P_NED.D = -2*(q2*q3+q1*q0)*V_xB;
			//ROS_INFO("V_xB speed, %.3f", V_xB); // ONLY FOR DEBUG		
			ROS_INFO("NED ODO speed, %.3f,%.3f,%.3f",d_hat_P_NED.N,d_hat_P_NED.E,d_hat_P_NED.D); // ONLY FOR DEBUG
			
			
			if (is_GPS_present == true && is_home_set == true)
			{
				/*if (HOME == true || GPS_first_msg == true)
				{
					P_HOME_GPS = inputGPS_;
					HOME = false;
					GPS_first_msg = false;
					ROS_ERROR("NEW HOME MSG: LAT, LON %.7f, %.7f",inputGPS_.latitude, inputGPS_.longitude); // ONLY FOR DEBUG
					P_HOME_ECEF = GPS2ECEF(P_HOME_GPS);
					// ROS_ERROR("P_HOME_ECEF, %.3f,%.3f,%.3f",P_HOME_ECEF.X,P_HOME_ECEF.Y,P_HOME_ECEF.Z); // ONLY FOR DEBUG
					cosPhi = cos(P_HOME_GPS.latitude/180.0*pi);
					sinPhi = sin(P_HOME_GPS.latitude/180.0*pi);
					cosLambda = cos(P_HOME_GPS.longitude/180.0*pi);
					sinLambda = sin(P_HOME_GPS.longitude/180.0*pi);
					// ROS_ERROR("c(Phi), s(Phi), c(Lam), s(Lam) %.3f, %.3f, %.3f, %.3f",cosPhi,sinPhi,cosLambda,sinLambda);
					// ROS_INFO("REF: SETTING_HOME");
					// pubHome_.publish(P_HOME_GPS);
				}*/
				P_ACTUAL_GPS.latitude = inputGPS_.latitude;
				P_ACTUAL_GPS.longitude = inputGPS_.longitude;
				P_ACTUAL_GPS.altitude = inputGPS_.hEll;
				// P_NED_GPS = GPS2NED(P_ACTUAL_GPS,P_HOME_ECEF);
			
				P_ACTUAL_ECEF = GPS2ECEF(P_ACTUAL_GPS);
				
				///////////////////////////////////
				ROS_WARN("P_HOME_ECEF, %.3f,%.3f,%.3f",P_HOME_ECEF.X,P_HOME_ECEF.Y,P_HOME_ECEF.Z); // ONLY FOR DEBUG
				ROS_WARN("P_ACTUAL_ECEF, %.3f,%.3f,%.3f",P_ACTUAL_ECEF.X,P_ACTUAL_ECEF.Y,P_ACTUAL_ECEF.Z); // ONLY FOR DEBUG
				DP_ECEF.X = P_ACTUAL_ECEF.X - P_HOME_ECEF.X;
				DP_ECEF.Y = P_ACTUAL_ECEF.Y - P_HOME_ECEF.Y; 
				DP_ECEF.Z = P_ACTUAL_ECEF.Z - P_HOME_ECEF.Z;
				ROS_WARN("DP_ECEF, %.3f,%.3f,%.3f",DP_ECEF.X,DP_ECEF.Y,DP_ECEF.Z); // ONLY FOR DEBUG

				t =  cosLambda* DP_ECEF.X + sinLambda* DP_ECEF.Y;				
				P_ACTUAL_NED.N = -sinPhi * t + cosPhi * DP_ECEF.Z;
				P_ACTUAL_NED.E = -sinLambda * DP_ECEF.X + cosLambda * DP_ECEF.Y;
				P_ACTUAL_NED.D = -(cosPhi * t + sinPhi * DP_ECEF.Z);
				P_NED_GPS = P_ACTUAL_NED;
 				
				ROS_INFO("NED GPS, %.3f,%.3f,%.3f",P_NED_GPS.N,P_NED_GPS.E,P_NED_GPS.D); // ONLY FOR DEBUG
				////////////////////////////////////
				ROS_INFO("NED errors, %.3f,%.3f,%.3f",P_NED_GPS.N-hat_P_NED.N,P_NED_GPS.E-hat_P_NED.E,P_NED_GPS.D-hat_P_NED.D); // ONLY FOR DEBUG
				// d_hat_P_NED = R_B2NED*[V_xB; 0; 0]+K*(P_NED_GPS-hat_P_NED);
				d_hat_P_NED.N = d_hat_P_NED.N + KN*(P_NED_GPS.N-hat_P_NED.N);
				d_hat_P_NED.E = d_hat_P_NED.E + KE*(P_NED_GPS.E-hat_P_NED.E);
				d_hat_P_NED.D = d_hat_P_NED.D + KD*(P_NED_GPS.D-hat_P_NED.D);
				ROS_INFO("NED ODO-GPS speed, %.3f,%.3f,%.3f",d_hat_P_NED.N,d_hat_P_NED.E,d_hat_P_NED.D); // ONLY FOR DEBUG
				
				is_GPS_present = false;
			}

			ROS_WARN("OLD_HAT_P_NED, %.3f,%.3f,%.3f",hat_P_NED.N,hat_P_NED.E,hat_P_NED.D); // ONLY FOR DEBUG
			hat_P_NED.N = d_hat_P_NED.N*DT+hat_P_NED.N; 
			hat_P_NED.E = d_hat_P_NED.E*DT+hat_P_NED.E; 
            hat_P_NED.D = d_hat_P_NED.D*DT+hat_P_NED.D; 
			ROS_WARN("NEW_HAT_P_NED, %.3f,%.3f,%.3f",hat_P_NED.N,hat_P_NED.E,hat_P_NED.D); // ONLY FOR DEBUG
			
			/*outputEstimatedPosition_.N = hat_P_NED.N;
			outputEstimatedPosition_.E = hat_P_NED.E;
			outputEstimatedPosition_.D = hat_P_NED.D;*/
			
			double p_lat, double p_lon, double p_alt
			get_pos_WGS84_from_NED (double *p_lat, double *p_lon, double *p_alt, double hat_P_NED.N, double hat_P_NED.E, double -hat_P_NED.D double P_HOME_GPS.latitude, double P_HOME_GPS.longitude, double P_HOME_GPS.hEll)
			// Publish the estimated position
			outputEstimatedPosition_.latitude = p_lat;
			outputEstimatedPosition_.longitude = p_lon;
			outputEstimatedPosition_.altitude = p_alt;
			pubEstimatedPosition_.publish(outputEstimatedPosition_);
			
		}

		void run()
		{
			ros::Rate loop_rate(rate);
			
			while (ros::ok())
			{
				ROS_INFO_ONCE("Position_Estimator: RUNNING");
				it_num = it_num+1; // ONLY FOR DEBUG
				ROS_INFO("Iterations number, %f", it_num); // ONLY FOR DEBUG
				PositionEstimation_Handle();
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

	protected:
		/*state here*/
		ros::NodeHandle n_;

		// Subscribers
		ros::Subscriber subFromImu_;
		sensor_msgs::Imu inputImu_;
		
		ros::Subscriber subFromGPS_;
		custom_msgs::gnssSample inputGPS_;
		geographic_msgs::GeoPoint P_ACTUAL_GPS;

		//ros::Subscriber subFromOdo_;
		//nav_msgs::Odometry inputOdo_;

		ros::Subscriber subFromHome_;
		geographic_msgs::GeoPoint inputHome_;
		geographic_msgs::GeoPoint P_HOME_GPS;

		ros::Subscriber subFromTrackSpeed_;
		donkey_rover::Rover_Track_Speed inputTrackSpeed_;	
		
		// Publishers
        ros::Publisher pubEstimatedPosition_;
		geographic_msgs::GeoPoint outputEstimatedPosition_;
       
		int rate = 10;
		float DT = 0.1;// 1/rate;

	private:
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_estimator");
	ros::NodeHandle node;

	PosEstimNodeClass PosEstimNode(node);

	PosEstimNode.run();
	return 0;
}

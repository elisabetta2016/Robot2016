#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "custom_msgs/gnssSample.h"
#include "sherpa_msgs/GeoPoint.h"
#include "std_msgs/Float64.h"
#include "math.h"
#include "donkey_rover/Rover_Track_Speed.h"
#include <wgs84_ned_lib/wgs84_ned_lib.h> 
#include <sherpa_msgs/Cmd.h> // input
#include "sherpa_msgs/MMS_status.h"
#include "sensor_msgs/MagneticField.h"
#include "sherpa_msgs/Attitude.h"

// #define NO_HOME 10

// TODO here include the message definitions
const int rate = 10;
const float DT = 0.1;// 1/rate;
bool Attitude_Body_Speed_ON;
bool Attitude_LPF_ON;
bool is_GPS_present = false;
bool is_Imu_present = false;
bool is_Magnetometer_present = false;
bool HOME = false;
float GPS_msg_count = 0;
bool GPS_first_msg = false;
//const float M_PI = 3.1415926535;
//float it_num = 1;
float VR = 0;
float VL = 0;
float V_yB = 0;
float V_th = 0.01;
bool flag_init = true;
bool is_home_set = false;
float cosPhi,sinPhi,cosLambda,sinLambda,t;
const double A = 6378137;    //major semiaxis
const double B = 6356752.3124;    //minor semiaxis
const double e = 0.0816733743281685; // first eccentricity
float R_B2ENU_11, R_B2ENU_12, R_B2ENU_13; // from body to ENU
float R_B2ENU_21, R_B2ENU_22, R_B2ENU_23; // from body to ENU
float R_B2ENU_31, R_B2ENU_32, R_B2ENU_33; // from body to ENU
float hat_R_B2ENU_11, hat_R_B2ENU_12, hat_R_B2ENU_13; // from body to ENU
float hat_R_B2ENU_21, hat_R_B2ENU_22, hat_R_B2ENU_23; // from body to ENU
float hat_R_B2ENU_31, hat_R_B2ENU_32, hat_R_B2ENU_33; // from body to ENU
double Prop_R_B2ENU_11, Prop_R_B2ENU_12, Prop_R_B2ENU_13;
double Prop_R_B2ENU_21, Prop_R_B2ENU_22, Prop_R_B2ENU_23;
double Prop_R_B2ENU_31, Prop_R_B2ENU_32, Prop_R_B2ENU_33;
// const double b = 10; // rover reference point [m]
double Kx; // EAST
double Ky; // NORTH
double Kz; // UP
double Kx_; // EAST
double Ky_; // NORTH
double Kz_; // UP
double K_LPF; // Attitude Low Pass Filter G(S) = K_LPF/(S+K_LPF)
double psi, theta, phi; // ENU psi = z, theta = y, phi = x
double q0, q1, q2, q3; 
// Initializazion Magnetic Field
double H_E = -1050.3; // [nT]
double H_N = 22586.3; // [nT]
double H_U = -40822.5; // [nT]
double H_norm = sqrt(pow(H_E,2)+pow(H_N,2)+pow(H_U,2));			
/*H_E = H_E/H_norm; // normalized
H_N = H_N/H_norm; // normalized
H_U = H_U/H_norm; // normalized*/
// Initializazion Gravitational Field
double g_E =  0.0;
double g_N =  0.0;
double g_U = -1.0; // normalized

/* vector product S = Magnetic x Gravity (both normalized)
 |  i   j   k  |
 | H_E H_N H_U |
 |  0   0   -1 |
*/
double S_E = -H_N;
double S_N =  H_E;
double S_U =  0.0;
double S_norm = sqrt(pow(S_E,2)+pow(S_N,2)+pow(S_U,2));	
/*S_E =  S_E/S_norm; // normalized
S_N =  S_N/S_norm; // normalized
S_U =  S_U/S_norm; // normalized*/

/* vector product M = Magnetic x S (both normalized)
 |  i   j   k  |
 | H_E H_N H_U |
 | S_E S_N  0  |
*/			
double M_East;// = -S_N*H_U;
double M_N;// =  S_E*H_U;
double M_U;// =  H_E*S_N-S_E*H_N;
double M_norm;// =  sqrt(pow(M_East,2)+pow(M_N,2)+pow(M_U,2));
/*M_East = M_East/M_norm; // normalized
M_N = M_N/M_norm; // normalized
M_U = M_U/M_norm; // normalized		*/	

/* A matrix = [H|S|M]
	| H_E S_E M_East |
A = | H_N S_N M_N |
	| H_U S_U M_U |
*/
			
// int currentState = NO_HOME;

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
			// subFromMmsStatus_ = n_.subscribe("/mms_status", 10, &PosEstimNodeClass::readMmsStatusMessage,this);
			subFromMagnetometer_ =n_.subscribe("/mti/sensor/magnetic", 10, &PosEstimNodeClass::readMagnetometerMessage,this);     
			// publishers
			pubEstimatedPosition_= n_.advertise<sherpa_msgs::GeoPoint>("/estimated_position", 10);
			// pubToCmd_ = n_.advertise<sherpa_msgs::Cmd>("/sent_command", 10);
			pubAttitude_ = n_.advertise<sherpa_msgs::Attitude>("/attitude",10);
			
			
			// Initializazion Magnetic Field			
			H_E = H_E/H_norm; // normalized
			H_N = H_N/H_norm; // normalized
			H_U = H_U/H_norm; // normalized
			
			S_E =  S_E/S_norm; // normalized
			S_N =  S_N/S_norm; // normalized
			S_U =  S_U/S_norm; // normalized
			
			M_East 	= -S_N*H_U;
			M_N 	=  S_E*H_U;
			M_U 	=  H_E*S_N-S_E*H_N;
			M_norm 	=  sqrt(pow(M_East,2)+pow(M_N,2)+pow(M_U,2));
			M_East 	= M_East/M_norm; // normalized
			M_N 	= M_N/M_norm; // normalized
			M_U 	= M_U/M_norm; // normalized
		}

		/*void readMmsStatusMessage(const sherpa_msgs::MMS_status::ConstPtr& msg)
		{
			inputMmsStatus_.mms_state=msg->mms_state;
			//inputMmsStatus_.target_ref_frame=msg->target_ref_frame;
			//ROS_INFO("REF: MMS_status received %d", inputMmsStatus_.mms_state);
			currentState = inputMmsStatus_.mms_state;
			//new_state = true;
			ROS_INFO("POS_ESTIM: MMS STATUS RECEIVED: %d", currentState);
		}*/	
		
		void readMagnetometerMessage(const sensor_msgs::MagneticField::ConstPtr& msg)
		{
			inputMagnetometer_.x = msg->magnetic_field.x;
			inputMagnetometer_.y = msg->magnetic_field.y;
			inputMagnetometer_.z = msg->magnetic_field.z;
		
			is_Magnetometer_present = true;
		}
		
		void readImuMessage(const sensor_msgs::Imu::ConstPtr& msg)
		{
			inputImu_.linear_acceleration.x = msg->linear_acceleration.x;
			inputImu_.linear_acceleration.y = msg->linear_acceleration.y;
			inputImu_.linear_acceleration.z = msg->linear_acceleration.z;
			inputImu_.angular_velocity.x = msg->angular_velocity.x;
			inputImu_.angular_velocity.y = msg->angular_velocity.y;
			inputImu_.angular_velocity.z = msg->angular_velocity.z;
			
			is_Imu_present = true;
			
		  /*inputImu_.orientation.w = msg->orientation.w; 
			inputImu_.orientation.x = msg->orientation.x; 
			inputImu_.orientation.y = msg->orientation.y; 
			inputImu_.orientation.z = msg->orientation.z;*/
			}

		void readGPSMessage(const custom_msgs::gnssSample::ConstPtr& msg) 				
		{	
			inputGPS_.heading = msg->heading; 
			inputGPS_.hMsl = msg->hMsl; 
			inputGPS_.hEll = msg->hEll; 
			inputGPS_.latitude = msg->latitude;
			inputGPS_.longitude = msg->longitude;
			inputGPS_.vel.x = msg->vel.x; //East
			inputGPS_.vel.y = msg->vel.y; //North
			inputGPS_.vel.z = msg->vel.z; //Up
		
			//ROS_INFO("NEW GPS MSG, LAT, LON %.7f, %.7f",inputGPS_.latitude, inputGPS_.longitude); // ONLY FOR DEBUG
		
			is_GPS_present = true;
			GPS_msg_count = GPS_msg_count+1;
			//ROS_INFO("GPS_msg_count %.0f",GPS_msg_count); // ONLY FOR DEBUG
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
			
			inputTrackSpeed_.Front_Right_Track_Speed = msg->Front_Right_Track_Speed;
			inputTrackSpeed_.Front_Left_Track_Speed = msg->Front_Left_Track_Speed;
			inputTrackSpeed_.Rear_Right_Track_Speed = msg->Rear_Right_Track_Speed;
			inputTrackSpeed_.Rear_Left_Track_Speed = msg->Rear_Left_Track_Speed;
			
			VR = (inputTrackSpeed_.Front_Right_Track_Speed+inputTrackSpeed_.Rear_Right_Track_Speed)/2.0;
			VL = (inputTrackSpeed_.Front_Left_Track_Speed+inputTrackSpeed_.Rear_Left_Track_Speed)/2.0;
			V_yB = (VR+VL)/2.0; // rover x-body speed in m/s
			
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
			P_HOME_ECEF = GPS2ECEF(P_HOME_GPS);
			is_home_set = true;
		}
		
		/*class v_ENU
		{ // this class defines a ENU vector
			public:
			float E;
			float N;
			float U;
			v_ENU()
			{
				N = 0.0; E = 0.0; U = 0.0;
			}
		};*/
		
		/*class v_ECEF
		{ // this class defines a ENU vector
			public:
			float X;
			float Y;
			float Z;
		};*/

		
		geometry_msgs::Vector3 GPS2ECEF(sherpa_msgs::GeoPoint P_ACTUAL_GPS_)
		{
			geometry_msgs::Vector3 P_ACTUAL_ECEF_;
			float N;

			// const float A = 6378137.0;
			// const float B = 6356752.3142;
			// const float M_PI = 3.1415926535;
			const double e = 0.0816733743281685; // first eccentricity
			
			cosPhi = cos(P_ACTUAL_GPS_.latitude/180.0*M_PI);
			sinPhi = sin(P_ACTUAL_GPS_.latitude/180.0*M_PI);
			cosLambda = cos(P_ACTUAL_GPS_.longitude/180.0*M_PI);
			sinLambda = sin(P_ACTUAL_GPS_.longitude/180.0*M_PI);
			//ROS_INFO("c(Phi), s(Phi), c(Lam), s(Lam) %.3f, %.3f, %.3f, %.3f",cosPhi,sinPhi,cosLambda,sinLambda);
			// e = sqrt(1.0-(pow(b,2))/(pow(a,2)));
			N = A/sqrt(1.0-pow(e,2)*pow(sinPhi,2));
			//ROS_INFO("e, N %.3f, %.3f",e,N);			
			P_ACTUAL_ECEF_.x = (N+P_ACTUAL_GPS_.altitude)*cosPhi*cosLambda;
			P_ACTUAL_ECEF_.y = (N+P_ACTUAL_GPS_.altitude)*cosPhi*sinLambda;
			P_ACTUAL_ECEF_.z = (((pow(B,2)/pow(A,2))*N+P_ACTUAL_GPS_.altitude))*sinPhi;			
			//ROS_INFO("P_ECEF X, Y, Z %f, %f, %f",P_ACTUAL_ECEF.X, P_ACTUAL_ECEF.Y, P_ACTUAL_ECEF.Z);
			return P_ACTUAL_ECEF_;
		}
		
		void PositionEstimation_Handle()
		{ 
/*			if (is_Magnetometer_present && !is_Imu_present)
			{
				// 2D based only on the magnetometer
				double inv_det = 1/(pow(H_E,2)+pow(H_N,2));
				
				double cos_psi   = inv_det*(H_E*inputMagnetometer_.x+H_N*inputMagnetometer_.y);
				double sin_psi   = inv_det*(H_N*inputMagnetometer_.x-H_E*inputMagnetometer_.y);
				
				psi   = atan2(sin_psi,cos_psi);
				theta = 0.0;
				phi   = 0.0;

				R_B2ENU_11 = cos_psi;
				R_B2ENU_12 = -sin_psi; 
				R_B2ENU_13 = 0.0;
				R_B2ENU_21 = sin_psi;
				R_B2ENU_22 = cos_psi; 
				R_B2ENU_23 = 0.0;
				R_B2ENU_31 = -0.0;
				R_B2ENU_32 = 0.0; 
				R_B2ENU_33 = 1.0;				
				
				q0 = cos(psi/2);
				q1 = 0.0;
				q2 = 0.0;
				q3 = sin(psi/2);

				// ROS_INFO("POS: psi_magn [rad] = %f",psi);//*180.0/M_PI);
				
 				outputAttitude_.euler.x = phi;
				outputAttitude_.euler.y = theta;
				outputAttitude_.euler.z = psi;
				outputAttitude_.quaternion.w = q0;
				outputAttitude_.quaternion.x = q1;
				outputAttitude_.quaternion.y = q2;
				outputAttitude_.quaternion.z = q3;
				outputAttitude_.R11 = R_B2ENU_11;
				outputAttitude_.R12 = R_B2ENU_21;
				outputAttitude_.R13 = R_B2ENU_31;
				outputAttitude_.R21 = R_B2ENU_12;
				outputAttitude_.R22 = R_B2ENU_22;
				outputAttitude_.R23 = R_B2ENU_32;
				outputAttitude_.R31 = R_B2ENU_13;
				outputAttitude_.R32 = R_B2ENU_23;
				outputAttitude_.R33 = R_B2ENU_33;
				pubAttitude_.publish(outputAttitude_); 	
			}*/
/*			if (is_Magnetometer_present && is_Imu_present)
			{
			
			// Magnetometer normalizazion
			double h_norm = sqrt(pow(inputMagnetometer_.x,2)+pow(inputMagnetometer_.y,2)+pow(inputMagnetometer_.z,2));			
			double h_x = inputMagnetometer_.x/h_norm; // normalized
			double h_y = inputMagnetometer_.y/h_norm; // normalized
			double h_z = inputMagnetometer_.z/h_norm; // normalized			
			// Accelerometer normalizazion
			double a_norm = sqrt(pow(inputImu_.linear_acceleration.x,2)+pow(inputImu_.linear_acceleration.y,2)+pow(inputImu_.linear_acceleration.z,2));			
			double a_x = -inputImu_.linear_acceleration.x/a_norm; // normalized (minus becasue the accelerometer measures -g)
			double a_y = -inputImu_.linear_acceleration.y/a_norm; // normalized (minus becasue the accelerometer measures -g)
			double a_z = -inputImu_.linear_acceleration.z/a_norm; // normalized (minus becasue the accelerometer measures -g)

			// vector product s = Magnetic x Gravity (both normalized)
			// |  i   j   k  |
			// | h_x h_y h_z |
			// | a_x a_y a_z |
			
			double s_x =  (h_y*a_z-a_y*h_z);
			double s_y = -(h_x*a_z-a_x*h_z);
			double s_z =  (h_x*a_y-a_x*h_y);
			double s_norm = sqrt(pow(s_x,2)+pow(s_y,2)+pow(s_z,2));
			s_x =  s_x/s_norm; // normalized
			s_y =  s_y/s_norm; // normalized
			s_z =  s_z/s_norm; // normalized			

			// vector product m = Magnetic x s (both normalized)
			// |  i   j   k  |
			// | h_x h_y h_z |
			// | s_x s_y s_z |
						
			double m_x =  (h_y*s_z-s_y*h_z);
			double m_y = -(h_x*s_z-s_x*h_z);
			double m_z =  (h_x*s_y-s_x*h_y);
			double m_norm = sqrt(pow(m_x,2)+pow(m_y,2)+pow(m_z,2));
			m_x =  m_x/m_norm; // normalized
			m_y =  m_y/m_norm; // normalized
			m_z =  m_z/m_norm; // normalized
			
			// rotation matrix from Body to ENU
			// R_B2ENU = [H|S|M]*[h|s|m]'
			R_B2ENU_11 = H_E*h_x + S_E*s_x + M_East*m_x; 
			R_B2ENU_12 = H_E*h_y + S_E*s_y + M_East*m_y;
			R_B2ENU_13 = H_E*h_z + S_E*s_z + M_East*m_z;
			R_B2ENU_21 = H_N*h_x + S_N*s_x + M_N*m_x; 
			R_B2ENU_22 = H_N*h_y + S_N*s_y + M_N*m_y;
			R_B2ENU_23 = H_N*h_z + S_N*s_z + M_N*m_z;
			R_B2ENU_31 = H_U*h_x + S_U*s_x + M_U*m_x; 
			R_B2ENU_32 = H_U*h_y + S_U*s_y + M_U*m_y;
			R_B2ENU_33 = H_U*h_z + S_U*s_z + M_U*m_z; 
			
			hat_R_B2ENU_11 = DT*K_LPF*(R_B2ENU_11-hat_R_B2ENU_11)+hat_R_B2ENU_11;
			hat_R_B2ENU_12 = DT*K_LPF*(R_B2ENU_12-hat_R_B2ENU_12)+hat_R_B2ENU_12;
			hat_R_B2ENU_13 = DT*K_LPF*(R_B2ENU_13-hat_R_B2ENU_13)+hat_R_B2ENU_13;
			hat_R_B2ENU_21 = DT*K_LPF*(R_B2ENU_21-hat_R_B2ENU_21)+hat_R_B2ENU_21;
			hat_R_B2ENU_22 = DT*K_LPF*(R_B2ENU_22-hat_R_B2ENU_22)+hat_R_B2ENU_22;
			hat_R_B2ENU_23 = DT*K_LPF*(R_B2ENU_23-hat_R_B2ENU_23)+hat_R_B2ENU_23;			
			hat_R_B2ENU_31 = DT*K_LPF*(R_B2ENU_31-hat_R_B2ENU_31)+hat_R_B2ENU_31;
			hat_R_B2ENU_32 = DT*K_LPF*(R_B2ENU_32-hat_R_B2ENU_32)+hat_R_B2ENU_32;
			hat_R_B2ENU_33 = DT*K_LPF*(R_B2ENU_33-hat_R_B2ENU_33)+hat_R_B2ENU_33;
			
			
			// rotation angles from Body to ENU = 3(z,psi) 2(y,theta) 1(x,phi)
			psi   = atan(R_B2ENU_21/R_B2ENU_11);
			theta = sin(-R_B2ENU_31);
			phi   = atan(R_B2ENU_32/R_B2ENU_33);

			// quaternion from Body to ENU: q0=scalar, (q1,q2,q3)=vector
			double cos_PHI = (0.5*(R_B2ENU_11+R_B2ENU_22+R_B2ENU_33-1));
			double PHI = acos(cos_PHI);
			double sin_PHI = sin(PHI);
			double sin_PHI_2 = sin(PHI/2);
			double e1 = -(R_B2ENU_23-R_B2ENU_32)/(2*sin_PHI);
			double e2 = -(R_B2ENU_31-R_B2ENU_13)/(2*sin_PHI);
			double e3 = -(R_B2ENU_12-R_B2ENU_21)/(2*sin_PHI);
			
			q0 =    cos(PHI/2); // scalar
			q1 = e1*sin_PHI_2; // vector x
			q2 = e2*sin_PHI_2; // vector y
			q3 = e3*sin_PHI_2; // vector z
			
			outputAttitude_.euler.x = phi;
			outputAttitude_.euler.y = theta;
			outputAttitude_.euler.z = psi;
			outputAttitude_.quaternion.w = q0;
			outputAttitude_.quaternion.x = q1;
			outputAttitude_.quaternion.y = q2;
			outputAttitude_.quaternion.z = q3;
			outputAttitude_.R11 = R_B2ENU_11;
			outputAttitude_.R12 = R_B2ENU_21;
			outputAttitude_.R13 = R_B2ENU_31;
			outputAttitude_.R21 = R_B2ENU_12;
			outputAttitude_.R22 = R_B2ENU_22;
			outputAttitude_.R23 = R_B2ENU_32;
			outputAttitude_.R31 = R_B2ENU_13;
			outputAttitude_.R32 = R_B2ENU_23;
			outputAttitude_.R33 = R_B2ENU_33;
			pubAttitude_.publish(outputAttitude_);			
			}*/
			
			if (is_Magnetometer_present)
			{
				if(!is_Imu_present)
				{
					// 2D based only on the magnetometer
					double inv_det = 1/(pow(H_E,2)+pow(H_N,2));
					
					double cos_psi   = inv_det*(H_E*inputMagnetometer_.x+H_N*inputMagnetometer_.y);
					double sin_psi   = inv_det*(H_N*inputMagnetometer_.x-H_E*inputMagnetometer_.y);
					
					psi   = atan2(sin_psi,cos_psi);
					theta = 0.0;
					phi   = 0.0;

					R_B2ENU_11 = cos_psi;
					R_B2ENU_12 = -sin_psi; 
					R_B2ENU_13 = 0.0;
					R_B2ENU_21 = sin_psi;
					R_B2ENU_22 = cos_psi; 
					R_B2ENU_23 = 0.0;
					R_B2ENU_31 = -0.0;
					R_B2ENU_32 = 0.0; 
					R_B2ENU_33 = 1.0;	
				}
				if(is_Imu_present)
				{
					// Magnetometer normalizazion
					double h_norm = sqrt(pow(inputMagnetometer_.x,2)+pow(inputMagnetometer_.y,2)+pow(inputMagnetometer_.z,2));			
					double h_x = inputMagnetometer_.x/h_norm; // normalized
					double h_y = inputMagnetometer_.y/h_norm; // normalized
					double h_z = inputMagnetometer_.z/h_norm; // normalized			
					// Accelerometer normalizazion
					double a_norm = sqrt(pow(inputImu_.linear_acceleration.x,2)+pow(inputImu_.linear_acceleration.y,2)+pow(inputImu_.linear_acceleration.z,2));			
					double a_x = -inputImu_.linear_acceleration.x/a_norm; // normalized (minus becasue the accelerometer measures -g)
					double a_y = -inputImu_.linear_acceleration.y/a_norm; // normalized (minus becasue the accelerometer measures -g)
					double a_z = -inputImu_.linear_acceleration.z/a_norm; // normalized (minus becasue the accelerometer measures -g)


					/* vector product s = Magnetic x Gravity (both normalized)
					 |  i   j   k  |
					 | h_x h_y h_z |
					 | a_x a_y a_z |
					*/
					double s_x =  (h_y*a_z-a_y*h_z);
					double s_y = -(h_x*a_z-a_x*h_z);
					double s_z =  (h_x*a_y-a_x*h_y);
					double s_norm = sqrt(pow(s_x,2)+pow(s_y,2)+pow(s_z,2));
					s_x =  s_x/s_norm; // normalized
					s_y =  s_y/s_norm; // normalized
					s_z =  s_z/s_norm; // normalized			

					/* vector product m = Magnetic x s (both normalized)
					 |  i   j   k  |
					 | h_x h_y h_z |
					 | s_x s_y s_z |
					*/			
					double m_x =  (h_y*s_z-s_y*h_z);
					double m_y = -(h_x*s_z-s_x*h_z);
					double m_z =  (h_x*s_y-s_x*h_y);
					double m_norm = sqrt(pow(m_x,2)+pow(m_y,2)+pow(m_z,2));
					m_x =  m_x/m_norm; // normalized
					m_y =  m_y/m_norm; // normalized
					m_z =  m_z/m_norm; // normalized
					
					// rotation matrix from Body to ENU
					// R_B2ENU = [H|S|M]*[h|s|m]'
					R_B2ENU_11 = H_E*h_x + S_E*s_x + M_East*m_x; 
					R_B2ENU_12 = H_E*h_y + S_E*s_y + M_East*m_y;
					R_B2ENU_13 = H_E*h_z + S_E*s_z + M_East*m_z;
					R_B2ENU_21 = H_N*h_x + S_N*s_x + M_N*m_x; 
					R_B2ENU_22 = H_N*h_y + S_N*s_y + M_N*m_y;
					R_B2ENU_23 = H_N*h_z + S_N*s_z + M_N*m_z;
					R_B2ENU_31 = H_U*h_x + S_U*s_x + M_U*m_x; 
					R_B2ENU_32 = H_U*h_y + S_U*s_y + M_U*m_y;
					R_B2ENU_33 = H_U*h_z + S_U*s_z + M_U*m_z; 
				}
				
				// FILTER THE DCM
				if (Attitude_LPF_ON)
				{
					if (Attitude_Body_Speed_ON && is_Imu_present)
					{
						double wg_x = inputImu_.angular_velocity.x;
						double wg_y = inputImu_.angular_velocity.y;
						double wg_z = inputImu_.angular_velocity.z;
						Prop_R_B2ENU_11 =  hat_R_B2ENU_12*wg_z - hat_R_B2ENU_13*wg_y;
						Prop_R_B2ENU_12 = -hat_R_B2ENU_11*wg_z + hat_R_B2ENU_13*wg_x;
						Prop_R_B2ENU_13 =  hat_R_B2ENU_11*wg_y - hat_R_B2ENU_12*wg_x;
						Prop_R_B2ENU_21 =  hat_R_B2ENU_22*wg_z - hat_R_B2ENU_23*wg_y;
						Prop_R_B2ENU_22 = -hat_R_B2ENU_21*wg_z + hat_R_B2ENU_23*wg_x;
						Prop_R_B2ENU_23 =  hat_R_B2ENU_21*wg_y - hat_R_B2ENU_22*wg_x;
						Prop_R_B2ENU_31 =  hat_R_B2ENU_32*wg_z - hat_R_B2ENU_33*wg_y;
						Prop_R_B2ENU_32 = -hat_R_B2ENU_31*wg_z + hat_R_B2ENU_33*wg_x;
						Prop_R_B2ENU_33 =  hat_R_B2ENU_31*wg_y - hat_R_B2ENU_32*wg_x;
					}
					else
					{
						Prop_R_B2ENU_11 =  0.0;
						Prop_R_B2ENU_12 =  0.0;
						Prop_R_B2ENU_13 =  0.0;
						Prop_R_B2ENU_21 =  0.0;
						Prop_R_B2ENU_22 =  0.0;
						Prop_R_B2ENU_23 =  0.0;
						Prop_R_B2ENU_31 =  0.0;
						Prop_R_B2ENU_32 =  0.0;
						Prop_R_B2ENU_33 =  0.0;						
					}
					hat_R_B2ENU_11 = DT*(Prop_R_B2ENU_11+K_LPF*(R_B2ENU_11-hat_R_B2ENU_11))+hat_R_B2ENU_11;
					hat_R_B2ENU_12 = DT*(Prop_R_B2ENU_12+K_LPF*(R_B2ENU_12-hat_R_B2ENU_12))+hat_R_B2ENU_12;
					hat_R_B2ENU_13 = DT*(Prop_R_B2ENU_13+K_LPF*(R_B2ENU_13-hat_R_B2ENU_13))+hat_R_B2ENU_13;
					hat_R_B2ENU_21 = DT*(Prop_R_B2ENU_21+K_LPF*(R_B2ENU_21-hat_R_B2ENU_21))+hat_R_B2ENU_21;
					hat_R_B2ENU_22 = DT*(Prop_R_B2ENU_22+K_LPF*(R_B2ENU_22-hat_R_B2ENU_22))+hat_R_B2ENU_22;
					hat_R_B2ENU_23 = DT*(Prop_R_B2ENU_23+K_LPF*(R_B2ENU_23-hat_R_B2ENU_23))+hat_R_B2ENU_23;			
					hat_R_B2ENU_31 = DT*(Prop_R_B2ENU_31+K_LPF*(R_B2ENU_31-hat_R_B2ENU_31))+hat_R_B2ENU_31;
					hat_R_B2ENU_32 = DT*(Prop_R_B2ENU_32+K_LPF*(R_B2ENU_32-hat_R_B2ENU_32))+hat_R_B2ENU_32;
					hat_R_B2ENU_33 = DT*(Prop_R_B2ENU_33+K_LPF*(R_B2ENU_33-hat_R_B2ENU_33))+hat_R_B2ENU_33;


					
				}
				if (!Attitude_LPF_ON)
				{
					hat_R_B2ENU_11 = R_B2ENU_11;
					hat_R_B2ENU_12 = R_B2ENU_12;
					hat_R_B2ENU_13 = R_B2ENU_13;
					hat_R_B2ENU_21 = R_B2ENU_21;
					hat_R_B2ENU_22 = R_B2ENU_22;
					hat_R_B2ENU_23 = R_B2ENU_23;
					hat_R_B2ENU_31 = R_B2ENU_31;
					hat_R_B2ENU_32 = R_B2ENU_32;
					hat_R_B2ENU_33 = R_B2ENU_33;
				}
				
				// DETERMINE EULER AND QUATERNION
				// rotation angles from Body to ENU = 3(z,psi) 2(y,theta) 1(x,phi)
				psi   = atan(hat_R_B2ENU_21/hat_R_B2ENU_11);
				theta = sin(-hat_R_B2ENU_31);
				phi   = atan(hat_R_B2ENU_32/hat_R_B2ENU_33);
				// quaternion from Body to ENU: q0=scalar, (q1,q2,q3)=vector
				double cos_PHI = (0.5*(hat_R_B2ENU_11+hat_R_B2ENU_22+hat_R_B2ENU_33-1));
				double PHI = acos(cos_PHI);
				double sin_PHI = sin(PHI);
				double sin_PHI_2 = sin(PHI/2);
				double e1 = -(hat_R_B2ENU_23-hat_R_B2ENU_32)/(2*sin_PHI);
				double e2 = -(hat_R_B2ENU_31-hat_R_B2ENU_13)/(2*sin_PHI);
				double e3 = -(hat_R_B2ENU_12-hat_R_B2ENU_21)/(2*sin_PHI);
				q0 =    cos(PHI/2); // scalar
				q1 = e1*sin_PHI_2; // vector x
				q2 = e2*sin_PHI_2; // vector y
				q3 = e3*sin_PHI_2; // vector z
				double norm_q = sqrt(pow(q0,2)+pow(q1,2)+pow(q2,2)+pow(q3,2));
				q0 = q0/norm_q;
				q1 = q1/norm_q;
				q2 = q2/norm_q;
				q3 = q3/norm_q;
				
				// PUBLISH THE FILTERED ATTITUDE
				outputAttitude_.euler.x = phi;
				outputAttitude_.euler.y = theta;
				outputAttitude_.euler.z = psi;
				outputAttitude_.quaternion.w = q0;
				outputAttitude_.quaternion.x = q1;
				outputAttitude_.quaternion.y = q2;
				outputAttitude_.quaternion.z = q3;
				outputAttitude_.R11 = hat_R_B2ENU_11;
				outputAttitude_.R12 = hat_R_B2ENU_21;
				outputAttitude_.R13 = hat_R_B2ENU_31;
				outputAttitude_.R21 = hat_R_B2ENU_12;
				outputAttitude_.R22 = hat_R_B2ENU_22;
				outputAttitude_.R23 = hat_R_B2ENU_32;
				outputAttitude_.R31 = hat_R_B2ENU_13;
				outputAttitude_.R32 = hat_R_B2ENU_23;
				outputAttitude_.R33 = hat_R_B2ENU_33;
				pubAttitude_.publish(outputAttitude_);
			}
			
			if (flag_init == true)
			{
				hat_P_ENU.x = 0.0;
				hat_P_ENU.y = 0.0;
				hat_P_ENU.z = 0.0;
				d_hat_P_ENU.x = 0;
				d_hat_P_ENU.y = 0;
				d_hat_P_ENU.z = 0;					
				flag_init = false;
				// ROS_ERROR("hat_P_ENU(0) %f,%f,%f ",hat_P_ENU.x,hat_P_ENU.y,hat_P_ENU.z);
			}

			// K = filter gain
			if(V_yB < V_th) // this check only works if the rover is not sliding!
			{
				Kx = 0.0; // EAST
				Ky = 0.0; // NORTH
				Kz = 0.0; // UP
			}
			else
			{
				Kx = Kx_; // EAST
				Ky = Ky_; // NORTH
				Kz = Kz_; // UP
			}			

				
			/*if (!is_home_set && is_GPS_present)
			{
				outputEstimatedPosition_.latitude = inputGPS_.latitude;
				outputEstimatedPosition_.longitude = inputGPS_.longitude;
				outputEstimatedPosition_.altitude = inputGPS_.hEll;
				pubEstimatedPosition_.publish(outputEstimatedPosition_);

				outputCmd_.command = 179; 
				outputCmd_.param1  = 1; 
				outputCmd_.param2  = 0.0; 
				outputCmd_.param3  = 0.0; 
				outputCmd_.param4  = 0.0; 
				outputCmd_.param5  = 0.0; 
				outputCmd_.param6  = 0.0; 
				outputCmd_.param7  = 0.0; 
				outputCmd_.frame   = 0.0; 
				outputCmd_.seq     = 1; 			
				pubToCmd_.publish(outputCmd_);
				ROS_INFO("POS_ESTIM: SET HOME");
			}*/
			//else
			if (is_home_set && is_Magnetometer_present && is_GPS_present)
			{
				P_ACTUAL_GPS.latitude = inputGPS_.latitude;
				P_ACTUAL_GPS.longitude = inputGPS_.longitude;
				P_ACTUAL_GPS.altitude = inputGPS_.hEll;
				
				P_ACTUAL_ECEF = GPS2ECEF(P_ACTUAL_GPS);
				//ROS_INFO("P_ACTUAL_ECEF, %.3f,%.3f,%.3f",P_ACTUAL_ECEF.x,P_ACTUAL_ECEF.y,P_ACTUAL_ECEF.z); // ONLY FOR DEBUG
				//ROS_INFO("P_HOME_ECEF, %.3f,%.3f,%.3f",P_HOME_ECEF.x,P_HOME_ECEF.y,P_HOME_ECEF.z); // ONLY FOR DEBUG
				///////////////////////////////////
				//ROS_WARN("P_HOME_ECEF, %.3f,%.3f,%.3f",P_HOME_ECEF.X,P_HOME_ECEF.Y,P_HOME_ECEF.Z); // ONLY FOR DEBUG
				//ROS_WARN("P_ACTUAL_ECEF, %.3f,%.3f,%.3f",P_ACTUAL_ECEF.X,P_ACTUAL_ECEF.Y,P_ACTUAL_ECEF.Z); // ONLY FOR DEBUG
				DP_ECEF.x = P_ACTUAL_ECEF.x - P_HOME_ECEF.x;
				DP_ECEF.y = P_ACTUAL_ECEF.y - P_HOME_ECEF.y; 
				DP_ECEF.z = P_ACTUAL_ECEF.z - P_HOME_ECEF.z;
				//ROS_WARN("DP_ECEF, %.3f,%.3f,%.3f",DP_ECEF.X,DP_ECEF.Y,DP_ECEF.Z); // ONLY FOR DEBUG
				// ROS_INFO("DP_ECEF, %.3f,%.3f,%.3f",DP_ECEF.x,DP_ECEF.y,DP_ECEF.z); // ONLY FOR DEBUG
				t =  cosLambda* DP_ECEF.x + sinLambda* DP_ECEF.y;
				P_ACTUAL_ENU.x = -sinLambda * DP_ECEF.x + cosLambda * DP_ECEF.y;					
				P_ACTUAL_ENU.y = -sinPhi * t + cosPhi * DP_ECEF.z;
				P_ACTUAL_ENU.z = cosPhi * t + sinPhi * DP_ECEF.z;
				P_ENU_GPS = P_ACTUAL_ENU;
				
				//ROS_INFO("ENU GPS, %.3f,%.3f,%.3f",P_ENU_GPS.N,P_ENU_GPS.E,P_ENU_GPS.D); // ONLY FOR DEBUG
				////////////////////////////////////
				//ROS_INFO("ENU errors, %.3f,%.3f,%.3f",P_ENU_GPS.N-hat_P_ENU.N,P_ENU_GPS.E-hat_P_ENU.E,P_ENU_GPS.D-hat_P_ENU.D); // ONLY FOR DEBUG
				// d_hat_P_ENU = R_B2NED*[V_yB; 0; 0]+K*(P_ENU_GPS-hat_P_ENU);
				/*d_hat_P_ENU.x = R_12*V_yB;
				d_hat_P_ENU.y = R_22*V_yB;
				d_hat_P_ENU.z = R_32*V_yB;*/
				//ROS_INFO("P_ENU_GPS, %.3f,%.3f,%.3f",P_ENU_GPS.x,P_ENU_GPS.y,P_ENU_GPS.z); // ONLY FOR DEBUG
				//ROS_INFO("OLD_HAT_P_ENU, %.3f,%.3f,%.3f",hat_P_ENU.x,hat_P_ENU.y,hat_P_ENU.z); // ONLY FOR DEBUG
				/*d_hat_P_ENU.x = R_12*V_yB + Kx*(P_ENU_GPS.x-hat_P_ENU.x);
				d_hat_P_ENU.y = R_22*V_yB + Ky*(P_ENU_GPS.y-hat_P_ENU.y);
				d_hat_P_ENU.z = R_32*V_yB + Kz*(P_ENU_GPS.z-hat_P_ENU.z);*/
				d_hat_P_ENU.x = R_B2ENU_12*V_yB + Kx*(P_ENU_GPS.x-hat_P_ENU.x);
				d_hat_P_ENU.y = R_B2ENU_22*V_yB + Ky*(P_ENU_GPS.y-hat_P_ENU.y);
				d_hat_P_ENU.z = 0;//R_B2ENU_32*V_yB + Kz*(P_ENU_GPS.z-hat_P_ENU.z);				
				//ROS_INFO("ENU ODO-GPS speed, %.3f,%.3f,%.3f",d_hat_P_ENU.E,d_hat_P_ENU.N,d_hat_P_ENU.U); // ONLY FOR DEBUG
				//ROS_INFO("d_hat_P_ENU, %.3f,%.3f,%.3f",d_hat_P_ENU.x,d_hat_P_ENU.y,d_hat_P_ENU.z); // ONLY FOR DEBUG
				//ROS_WARN("OLD_HAT_P_NED, %.3f,%.3f,%.3f",hat_P_ENU.E,hat_P_ENU.N,hat_P_ENU.U); // ONLY FOR DEBUG
				
				hat_P_ENU.x = d_hat_P_ENU.x*DT+hat_P_ENU.x;
				hat_P_ENU.y = d_hat_P_ENU.y*DT+hat_P_ENU.y;  
				hat_P_ENU.z = d_hat_P_ENU.z*DT+hat_P_ENU.z; 
				//ROS_WARN("NEW_HAT_P_ENU, %.3f,%.3f,%.3f",hat_P_ENU.x,hat_P_ENU.y,hat_P_ENU.z); // ONLY FOR DEBUG
				
				/*outputEstimatedPosition_.N = hat_P_ENU.N;
				outputEstimatedPosition_.E = hat_P_ENU.E;
				outputEstimatedPosition_.D = hat_P_ENU.D;*/
				
				//double p_lat;
				//double p_lon;
				//double p_alt;
				// get_pos_WGS84_from_NED( double *p_lat, double *p_lon, double *p_alt, hat_P_ENU.N, hat_P_ENU.E, -hat_P_ENU.D, P_HOME_GPS.latitude, P_HOME_GPS.longitude, P_HOME_GPS.altitude);
				// double e = sqrt(1-pow(B,2)/pow(A,2));
				// double lat_home_rad = P_HOME_GPS.latitude*M_PI/180.0f;
				// double lon_home_rad = P_HOME_GPS.longitude*M_PI/180.0f;
				double radius = A/sqrt(1-pow(e,2)*pow(sin(P_HOME_GPS.latitude*M_PI/180.0f),2));
				// ROS_INFO("radius %f",radius); // ONLY FOR DEBUG
				// Publish the estimated position
				outputEstimatedPosition_.longitude = hat_P_ENU.x/radius*180.0f/M_PI*cos(P_HOME_GPS.latitude*M_PI/180.0f) + P_HOME_GPS.longitude;
				outputEstimatedPosition_.latitude  = hat_P_ENU.y/radius*180.0f/M_PI + P_HOME_GPS.latitude;
				outputEstimatedPosition_.altitude  = 0;//P_HOME_GPS.altitude - hat_P_ENU.z; //z positive UP, negative DOWN, alt_home = hEll
				// outputEstimatedPosition_.latitude = inputGPS_.latitude;
				// outputEstimatedPosition_.longitude =  inputGPS_.longitude;
				// outputEstimatedPosition_.altitude =  inputGPS_.hEll;
				pubEstimatedPosition_.publish(outputEstimatedPosition_);
			}
		}

		void run()
		{
			ros::Rate loop_rate(rate);
			
			while (ros::ok())
			{
				ROS_INFO_ONCE("Position_Estimator: RUNNING");
				//it_num = it_num+1; // ONLY FOR DEBUG
				//ROS_INFO("Iterations number, %f", it_num); // ONLY FOR DEBUG
				ros::NodeHandle n("~");
				n.param("Filter_Gain_Kx", Kx_, 0.10);
				n.param("Filter_Gain_Kx", Ky_, 0.10);
				n.param("Filter_Gain_Kx", Kz_, 0.10);
				n.param("Attitude_LPF_ON", Attitude_LPF_ON, false);
 				n.param("K_LPF",K_LPF,0.1);
				n.param("Attitude_Body_Speed_ON", Attitude_Body_Speed_ON, false);
				ROS_INFO_ONCE("POS_ESTIM: Kx = %f",Kx_);
				ROS_INFO_ONCE("POS_ESTIM: Kx = %f",Ky_);
				ROS_INFO_ONCE("POS_ESTIM: Kx = %f",Kz_);
				ROS_INFO_ONCE("POS_ESTIM: Attitude_LPF_ON = %d",Attitude_LPF_ON);
				ROS_INFO_ONCE("POS_ESTIM: K_LPF = %f",K_LPF);
				ROS_INFO_ONCE("POS_ESTIM: Attitude_Body_Speed_ON = %d",Attitude_Body_Speed_ON);
				PositionEstimation_Handle();
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

	protected:
		/*state here*/
		ros::NodeHandle n_;

		// Subscribers
		/*ros::Subscriber subFromMmsStatus_;
		sherpa_msgs::MMS_status inputMmsStatus_;*/
		
		ros::Subscriber subFromImu_;
		sensor_msgs::Imu inputImu_;
		
		ros::Subscriber subFromGPS_;
		custom_msgs::gnssSample inputGPS_;
		sherpa_msgs::GeoPoint P_ACTUAL_GPS;

		//ros::Subscriber subFromOdo_;
		//nav_msgs::Odometry inputOdo_;

		ros::Subscriber subFromHome_;
		sherpa_msgs::GeoPoint inputHome_;
		sherpa_msgs::GeoPoint P_HOME_GPS;

		ros::Subscriber subFromTrackSpeed_;
		donkey_rover::Rover_Track_Speed inputTrackSpeed_;	
		
		ros::Subscriber subFromMagnetometer_;
		geometry_msgs::Vector3 inputMagnetometer_;
		
		// Publishers
        ros::Publisher pubEstimatedPosition_;
		sherpa_msgs::GeoPoint outputEstimatedPosition_;
	
	    ros::Publisher pubAttitude_;
		sherpa_msgs::Attitude outputAttitude_;
		
		/*ros::Publisher pubToCmd_;
		sherpa_msgs::Cmd outputCmd_;*/
		
		/*v_ENU  d_hat_P_ENU;
		v_ENU  P_ENU_GPS;
		v_ENU  P_ACTUAL_ENU;
		v_ENU  hat_P_ENU;
		v_ECEF P_HOME_ECEF;
		v_ECEF P_ACTUAL_ECEF;
		v_ECEF DP_ECEF;*/
		geometry_msgs::Vector3 d_hat_P_ENU,P_ENU_GPS,P_ACTUAL_ENU,hat_P_ENU,P_HOME_ECEF,P_ACTUAL_ECEF,DP_ECEF;

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

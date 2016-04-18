#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sherpa_msgs/GeoPoint.h"
#include "geometry_msgs/Vector3.h"
#include "sherpa_msgs/MMS_status.h"
#include "math.h"
#include "sensor_msgs/MagneticField.h"

bool is_Imu_available = false;
bool is_GPS_available = false;
bool is_target_available = false;
bool is_Magnetometer_present = false;

const double A = 6378137;    //major semiaxis
const double B = 6356752.3124;    //minor semiaxis
const double e = 0.0816733743281685; // first eccentricity
const double pi = 3.1415926535;
double b; // rover reference point [m]
double delta_position;
double radius = A; 
double R_11, R_12, R_13; 
double R_21, R_22, R_23;
double R_31, R_32, R_33;
double psi, theta, phi; // ENU psi = z, theta = y, phi = x
double mod_psi, cos_psi, sin_psi;

#define NO_HOME 10
#define SETTING_HOME 20
#define READY_TO_GO 30
#define PERFORMING_GO_TO 100
#define LEASHING 140
#define PAUSED 150
#define MANUAL_CNTRL 1000

class BFErrorNodeClass
{
	public:
			
		BFErrorNodeClass(ros::NodeHandle& node)
		{
			n_=node;

			//subscribers
			// subFromImu_ = n_.subscribe("/mti/sensor/imu", 10, &BFErrorNodeClass::readImuMessage,this);
			// subFromHome_= n_.subscribe("/home", 10, &BFErrorNodeClass::readHomeMessage,this);
			subFromCurrentPosition_= n_.subscribe("/estimated_position", 10, &BFErrorNodeClass::readPositionMessage,this);
			subFromTargetPosition_= n_.subscribe("/reference", 10, &BFErrorNodeClass::readTargetMessage,this);
			subFromMmsStatus_ = n_.subscribe("/mms_status", 10, &BFErrorNodeClass::readMmsStatusMessage,this);
			subFromMagnetometer_ =n_.subscribe("/mti/sensor/magnetic", 10, &BFErrorNodeClass::readMagnetometerMessage,this);
			
			// publishers
			pubTrackingError_ = n_.advertise<geometry_msgs::Vector3>("/body_error",10);	
			
			// initialize
			currentState = NO_HOME;
		}

		void readMagnetometerMessage(const sensor_msgs::MagneticField::ConstPtr& msg)
		{
			inputMagnetometer_.x = msg->magnetic_field.x;
			inputMagnetometer_.y = msg->magnetic_field.y;
			inputMagnetometer_.z = msg->magnetic_field.z;
			is_Magnetometer_present = true;
			
			double H_E = -1050.3; // [nT]
			double H_N = 22586.3; // [nT]
			double H_U = -40822.5; // [nT]
			
			double H_norm = sqrt(pow(H_E,2)+pow(H_N,2)+pow(H_U,2));			
			H_E = H_E/H_norm;
			H_N = H_N/H_norm;
			H_U = H_U/H_norm;
			
			double inv_det = 1/(pow(H_E,2)+pow(H_N,2));
			
			cos_psi   = inv_det*(H_E*inputMagnetometer_.x+H_N*inputMagnetometer_.y);
			sin_psi   = inv_det*(H_N*inputMagnetometer_.x-H_E*inputMagnetometer_.y);
			
			// psi = atan2(sin_psi,cos_psi);
			// ROS_INFO("psi [deg] = %f",psi*180.0/M_PI);
			
		}
		
		void readMmsStatusMessage(const sherpa_msgs::MMS_status::ConstPtr& msg)
		{
			inputMmsStatus_.mms_state=msg->mms_state;
			//inputMmsStatus_.target_ref_frame=msg->target_ref_frame;
			//ROS_INFO("REF: MMS_status received %d", inputMmsStatus_.mms_state);
			currentState = inputMmsStatus_.mms_state;
			//new_state = true;
			ROS_INFO("BFE: MMS STATUS RECEIVED: %d", currentState);
			BodyFrameError_Handle();
		}		
		
		/*void readImuMessage(const sensor_msgs::Imu::ConstPtr& msg)
		{
			inputImu_.orientation.w = msg->orientation.w; 
			inputImu_.orientation.x = msg->orientation.x; 
			inputImu_.orientation.y = msg->orientation.y; 
			inputImu_.orientation.z = msg->orientation.z;
			double q1,q2,q3,q0;
			q0 = inputImu_.orientation.w; // scalar component					   
			q1 = inputImu_.orientation.x;
			q2 = inputImu_.orientation.y;
			q3 = inputImu_.orientation.z;
			is_Imu_available = true;
			// ROTATION MATRIX FROM SENSOR TO ENU
			R_11 = 2*(q0*q0+q1*q1)-1;
			R_22 = 2*(q0*q0+q2*q2)-1;
			R_33 = 2*(q0*q0+q3*q3)-1;
			R_12 = 2*(q1*q2-q0*q3);
			R_21 = 2*(q1*q2+q0*q3);
			R_13 = 2*(q0*q2+q1*q3);
			R_31 = 2*(-q0*q2+q1*q3);
			R_23 = 2*(-q0*q1+q2*q3);
			R_32 = 2*(q0*q1+q2*q3);
			
			phi = atan(R_32/R_33);
			theta = -asin(R_31);
			psi = atan(R_21/R_11);
			mod_psi = sqrt(pow(R_21,2)+pow(R_11,2));
			cos_psi = R_11/mod_psi;
			sin_psi = R_21/mod_psi;
			
			BodyFrameError_Handle();
		}*/
		
		void readPositionMessage(const sherpa_msgs::GeoPoint::ConstPtr& msg) // TODO
		{
			P_ACTUAL_GPS = *msg;
			is_GPS_available = true;
			BodyFrameError_Handle();
		}
		
		void readTargetMessage(const sherpa_msgs::GeoPoint::ConstPtr& msg) // TODO
		{
			P_TARGET_GPS = *msg;	
			is_target_available = true;
			BodyFrameError_Handle();
		}
		
		void BodyFrameError_Handle()
		{
			outputTrackingError_.x = 0; // like EAST
			outputTrackingError_.y = 0; // like NORTH
			outputTrackingError_.z = 0; // like UP

			if ((is_GPS_available == true) && (is_Magnetometer_present == true) && (is_target_available == true))
			{
				//ROS_INFO("BFE: actual lat lon %f %f", P_ACTUAL_GPS.latitude, P_ACTUAL_GPS.longitude);
				//ROS_INFO("BFE: target lat lon %f %f", P_TARGET_GPS.latitude, P_TARGET_GPS.longitude);
				radius = A/sqrt(1-pow(e,2)*pow(sin(P_ACTUAL_GPS.latitude*pi/180.0f),2));
				P_ERROR.y  = (P_TARGET_GPS.latitude-P_ACTUAL_GPS.latitude)*radius*pi/180.0f; // NORTH
				P_ERROR.x  = (P_TARGET_GPS.longitude-P_ACTUAL_GPS.longitude)*radius*cos(P_ACTUAL_GPS.latitude*pi/180.0f)*pi/180.0f; // EAST
				P_ERROR.z  = 0;//P_TARGET_GPS.altitude-P_ACTUAL_GPS.altitude; // UP
				// ROS_INFO("radius %f",radius);
				// ROS_INFO("P_ERROR E, N, %f, %f",P_ERROR.x, P_ERROR.y);									
				// HERE USE THE TRANSPOSE OF THE ROTATION MATRIX TO GO FROM THE ENU TO THE SENSOR
				/*if ((currentState == PAUSED) || (currentState == READY_TO_GO))
				{
					delta_position = b;
				}
				else
				{
					delta_position = b;
				}*/
				delta_position = b;
				/*outputTrackingError_.x = R_11*P_ERROR.x + R_21*P_ERROR.y                  +0; // x_body
				outputTrackingError_.y = R_12*P_ERROR.x + R_22*P_ERROR.y                  -delta_position; // y_body
				outputTrackingError_.z =                                   R_33*P_ERROR.z +0; // z_body*/
				outputTrackingError_.x =   cos_psi*P_ERROR.x + sin_psi*P_ERROR.y                  +0; // x_body
				outputTrackingError_.y = - sin_psi*P_ERROR.x + cos_psi*P_ERROR.y                  -delta_position; // y_body
				outputTrackingError_.z = 0; // z_body
				// ROS_INFO("quaternion w, x, y, z %f, %f, %f, %f",q0, q1, q2 ,q3);

				if ((currentState == PAUSED) || (currentState == READY_TO_GO))
				{
				outputTrackingError_.x = 0.0;//  cos_psi*P_ERROR.x + sin_psi*P_ERROR.y                  +0; // x_body
				outputTrackingError_.y = 0.0;//- sin_psi*P_ERROR.x + cos_psi*P_ERROR.y                  -delta_position; // y_body
				outputTrackingError_.z = 0.0; // z_body
				}
				
				// is_Imu_available = false;
				// is_GPS_available = false;
				// is_target_available = false;
			}
			// ROS_INFO("Tracking Error Xb, Yb, %f, %f",outputTrackingError_.x, outputTrackingError_.y);
			pubTrackingError_.publish(outputTrackingError_);			
		}

		void run()
		{
			ROS_INFO_ONCE("BodyFrameError: RUNNING");
			ros::NodeHandle n("~");
			n.param("safety_distance", b, 10.0);
			ROS_INFO_ONCE("BFE: b = %f",b);
			ros::spin();
			
			/*ros::Rate loop_rate(rate);
			
			while (ros::ok())
			{
				ROS_INFO_ONCE("BodyFrameError: RUNNING");
				// it_num = it_num+1; // ONLY FOR DEBUG
				// ROS_INFO("Iterations number, %f", it_num); // ONLY FOR DEBUG
				BodyFrameError_Handle();
				ros::spinOnce();
				loop_rate.sleep();
			}*/
		}

	protected:
		/*state here*/
		ros::NodeHandle n_;

		// Subscribers
		ros::Subscriber subFromMmsStatus_;
		sherpa_msgs::MMS_status inputMmsStatus_;
		
		ros::Subscriber subFromImu_;
		sensor_msgs::Imu inputImu_;

		ros::Subscriber subFromHome_;
		sherpa_msgs::GeoPoint P_HOME_GPS;
		
		ros::Subscriber subFromCurrentPosition_;
		sherpa_msgs::GeoPoint P_ACTUAL_GPS;
		
		ros::Subscriber subFromTargetPosition_;
		sherpa_msgs::GeoPoint P_TARGET_GPS;	
	
		ros::Subscriber subFromMagnetometer_;
		geometry_msgs::Vector3 inputMagnetometer_;	
		
		// Publishers
        ros::Publisher pubTrackingError_;
		geometry_msgs::Vector3 outputTrackingError_;
		geometry_msgs::Vector3 P_ERROR;
		

		       
		//int rate = 10;
		//float DT = 0.1; //1/rate;
		
		// STATE INITIALIZATION
		int currentState;		

	private:
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "body_frame_error");
	ros::NodeHandle node;

	BFErrorNodeClass BFErrorNode(node);

	BFErrorNode.run();
	return 0;
}

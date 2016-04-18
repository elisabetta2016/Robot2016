#include "ros/ros.h"

#include <std_msgs/String.h> // input
#include <sherpa_msgs/Cmd.h>// output

uint16_t seq_number = 0;
bool is_follow_me = false;

class FakeDelegationNodeClass
{
	public:
		FakeDelegationNodeClass(ros::NodeHandle& node)
		{

			n_=node;

			//subscribers
			subFromFakeTST_=n_.subscribe("/string_cmd", 10, &FakeDelegationNodeClass::readFakeTSTMessage,this); 
			
			// publishers
			pubToCmd_ = n_.advertise<sherpa_msgs::Cmd>("/sent_command", 10);

		}

		void readFakeTSTMessage(const std_msgs::String::ConstPtr& msg)
		{
			// TODO
			inputFakeTST_.data = msg -> data; // TODO
			if (inputFakeTST_.data == "follow me")
				is_follow_me = true;
			else if (inputFakeTST_.data == "enough")
				is_follow_me = false;
		
			FakeDelagation_Handle();
		}

		
		void FakeDelagation_Handle()
		{
			if (is_follow_me)
			{
			  /*LEASHING START
				rostopic pub -1 /sent_command sherpa_msgs/Cmd -- 0 0 3 0 25 0 0 1 0 0 0 0 0 0*/
				seq_number++;
				outputCmd_.command = 25; // TODO
				outputCmd_.param1  = 1; // TODO
				outputCmd_.param2  = 0.0; // TODO
				outputCmd_.param3  = 0.0; // TODO
				outputCmd_.param4  = 0.0; // TODO
				outputCmd_.param5  = 0.0; // TODO
				outputCmd_.param6  = 0.0; // TODO
				outputCmd_.param7  = 0.0; // TODO
				outputCmd_.frame   = 0.0; // TODO
				outputCmd_.seq     = seq_number; // TODO
				
				pubToCmd_.publish(outputCmd_);
				ROS_INFO("FAKE_DELEGATION: START LEASHING");						
			}
			else if (!is_follow_me)
			{
			/*  LEASHING STOP
				rostopic pub -1 /sent_command sherpa_msgs/Cmd -- 0 0 3 0 25 0 0 0 0 0 0 0 0 0*/
				seq_number++;
				outputCmd_.command = 25; 
				outputCmd_.param1  = 0.0; 
				outputCmd_.param2  = 0.0; 
				outputCmd_.param3  = 0.0; 
				outputCmd_.param4  = 0.0; 
				outputCmd_.param5  = 0.0; 
				outputCmd_.param6  = 0.0;
				outputCmd_.param7  = 0.0; 
				outputCmd_.frame   = 0.0; 
				outputCmd_.seq     = seq_number; 
				
				pubToCmd_.publish(outputCmd_);
				ROS_INFO("FAKE_DELEGATION: STOP LEASHING");					
			}

		
		}

		void run()
		{
			ROS_INFO_ONCE("FAKE_DELEGATION: RUNNING");
			ros::spin();
		}

	protected:
	/*state here*/
	ros::NodeHandle n_;

	// Subscribers

	ros::Subscriber subFromFakeTST_;
	std_msgs::String inputFakeTST_;

	// Publishers
	ros::Publisher pubToCmd_;
	sherpa_msgs::Cmd outputCmd_;

	private:


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FakeDelegation");
	
	ros::NodeHandle node;

	FakeDelegationNodeClass FakeDelegationNode(node);
	FakeDelegationNode.run();
	
	return 0;
}
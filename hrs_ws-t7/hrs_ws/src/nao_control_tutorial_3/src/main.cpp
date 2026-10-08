#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/BlinkGoal.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Duration.h>

using namespace std;

bool stop_thread=false;

void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
		//ROS_INFO_STREAM("Spinning the thing!!");
	}
}


class Nao_control
{
public:
	// ros handler
	ros::NodeHandle nh_;

	// subscriber to bumpers states
	ros::Subscriber bumper_sub;

	// subscriber to head tactile states
	ros::Subscriber tactile_sub;

	//publisher for nao speech
	ros::Publisher speech_pub;

	//publisher for nao leds
	ros::Publisher leds_pub;

	//publisher for nao vocabulary parameters
	ros::Publisher voc_params_pub;

	//client for starting speech recognition
	ros::ServiceClient recog_start_srv;

	//client for stoping speech recognition
	ros::ServiceClient recog_stop_srv;

	// subscriber to speech recognition
	ros::Subscriber recog_sub;

	// publisher to nao walking
	ros::Publisher walk_pub;

	//subscriber for foot contact
	ros::Subscriber footContact_sub;

	boost::thread *spin_thread;

	/*******************HERE IS MINE*******************/
	int bumper_index = 0;
	int bumper_state = 0;
	bool bumper_touched = false;

	bool head_front_touched = false;
	bool head_middle_touched = false;


	Nao_control()
	{

		// subscribe to topic bumper and specify that all data will be processed by function bumperCallback
		bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
		tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

		leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

		voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

		recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

		recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCB, this);

		footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCB, this);

		walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

		stop_thread=false;
		spin_thread=new boost::thread(&spinThread);
	}
	~Nao_control()
	{
		stop_thread=true;
		sleep(1);
		spin_thread->join();
	}

	void footContactCB(const std_msgs::BoolConstPtr& contact)
	{
		/*
		 * TODO tutorial 3
		 */
	}

	void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
		/*
		 * TODO tutorial 3
		 */
	}

	void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
	{
		/*
		 * TODO tutorial 3
		 */
		
		bumper_index = bumperState->bumper;   // 0 for right, 1 for left
		bumper_state = bumperState->state;

	}

	void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
	{
		/*
		 * TODO tutorial 3
		 */
		if (tactileState->state == 1 && tactileState->button == 1)
		{
			std::vector<string> voc;
			voc.push_back("hello");
			voc.push_back("world");

			head_front_touched = true;
		}
		else if (tactileState->state == 1 && tactileState->button == 2)
		{
			head_middle_touched = true;
			head_front_touched = false;
		}
	}


	void main_loop(int mode)
	{
		ros::Rate rate_sleep(2);
		while(nh_.ok())
		{	
			if (mode==1)
			{
			// blinking
				if (bumper_state == 1 && bumper_index == 0)   // right
				{
					std::cout << "ahhhhh on right \n";
					naoqi_bridge_msgs::BlinkGoal blink_goal;
		 			std_msgs::ColorRGBA red;
		 			red.r = 1.0;
		 			red.g = 0.0;
		 			red.b = 0.0;
		 			red.a = 1.0;
		 			blink_goal.colors.push_back(red);
		 			blink_goal.bg_color.r = 0.0;
		 			blink_goal.bg_color.g = 0.0;
		 			blink_goal.bg_color.b = 0.0;
		 			blink_goal.bg_color.a = 0.0;
		 			ros::Duration duration(2.0);
		 			blink_goal.blink_duration = duration;
		 			blink_goal.blink_rate_mean = 0.5;
		 			blink_goal.blink_rate_sd = 0.25;
		 			naoqi_bridge_msgs::BlinkActionGoal blink_action_goal;
		 			blink_action_goal.goal = blink_goal;
		 			leds_pub.publish(blink_action_goal);
				}
				else if(bumper_state == 1 && bumper_index == 1)  //left
				{
					std::cout << "ahhhhh on left \n";
					naoqi_bridge_msgs::BlinkGoal blink_goal;
		 			std_msgs::ColorRGBA blue;
		 			blue.r = 0.0;
		 			blue.g = 1.0;
		 			blue.b = 0.0;
		 			blue.a = 1.0;
		 			blink_goal.colors.push_back(blue);
		 			blink_goal.bg_color.r = 0.0;
		 			blink_goal.bg_color.g = 0.0;
		 			blink_goal.bg_color.b = 0.0;
		 			blink_goal.bg_color.a = 0.0;
		 			ros::Duration duration(2);
		 			blink_goal.blink_duration = duration;
		 			blink_goal.blink_rate_mean = 0.5;
		 			blink_goal.blink_rate_sd = 0.25;
		 			naoqi_bridge_msgs::BlinkActionGoal blink_action_goal;
		 			blink_action_goal.goal = blink_goal;
		 			leds_pub.publish(blink_action_goal);
				}
			bumper_state = 0;
			}
			/*
			 * TODO tutorial 3
			 */
			else if (mode == 2)
			{
			// HERE!!!!!!!!!!!!!!
			if (head_front_touched == true)
			{
				std::cout<< "I'm touched, speak to me \n";
				
			}
			}
			rate_sleep.sleep();
		}
	}

	void walker(double x, double y, double theta)
	{
		/*
		 * TODO tutorial 3
		 */
	}

	void stopWalk()
	{
		/*
		 * TODO tutorial 3
		 */
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "tutorial_control");

	ros::NodeHandle n;
	ros::Rate rate_sleep(2);
	Nao_control ic;
	
	ic.main_loop(1);
	rate_sleep.sleep();
	return 0;

}

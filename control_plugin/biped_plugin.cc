#ifndef _BIPED_PLUGIN_HH_
#define _BIPED_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <string>
#include <random>
#include <chrono>
#include <zmq.hpp>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <mutex>

#include "torque_setpoint.hpp"
#include "leg_state.hpp"

#include <unistd.h>

#include <zcm/zcm-cpp.hpp>
#include <sys/types.h>

using namespace std;
using namespace std::chrono;

namespace gazebo
{
	gazebo::physics::ModelPtr model;

	std::thread leftLegStateThread;
	std::thread rightLegStateThread;
	std::thread leftLegTorqueThread;
	std::thread rightLegTorqueThread;

	gazebo::physics::JointPtr leftHip3Joint;
	gazebo::physics::JointPtr leftHip2Joint;
	gazebo::physics::JointPtr leftHip1Joint;
	gazebo::physics::JointPtr leftKneeJoint;
	gazebo::physics::JointPtr leftAnkleJoint;

	gazebo::physics::JointPtr rightHip3Joint;
	gazebo::physics::JointPtr rightHip2Joint;
	gazebo::physics::JointPtr rightHip1Joint;
	gazebo::physics::JointPtr rightKneeJoint;
	gazebo::physics::JointPtr rightAnkleJoint;

	double torqueApplyingInterval = 1000; //microseconds
	double statePublishingInterval = 1000; //microseconds

	zcm::ZCM zcm_context { "ipc" };

	const char* left_leg_state_channel = "left_leg_state";
	const char* right_leg_state_channel = "right_leg_state";

	const char* left_leg_torque_setpoint_channel = ".left_leg_torque_setpoint";
	const char* right_leg_torque_setpoint_channel = ".right_leg_torque_setpoint";

	const int udp_port = 4200;

	const int udp_buffer_size = 4096;

	class Handler
	{
		public:
			~Handler() {}

			void handleLeftLegTorqueSetpointMessage(const zcm::ReceiveBuffer* rbuf,
							const string& chan,
							const torque_setpoint *setpoint)
			{
				model->GetJointController()->SetForce(leftHip3Joint->GetScopedName(), setpoint->tau1);
				model->GetJointController()->SetForce(leftHip2Joint->GetScopedName(), setpoint->tau2);
				model->GetJointController()->SetForce(leftHip1Joint->GetScopedName(), setpoint->tau3);
				model->GetJointController()->SetForce(leftKneeJoint->GetScopedName(), setpoint->tau4);
				model->GetJointController()->SetForce(leftAnkleJoint->GetScopedName(), setpoint->tau5);

				std::cout << "Received left leg torque setpoint." << std::endl;
			}

			void handleRightLegTorqueSetpointMessage(const zcm::ReceiveBuffer* rbuf,
							const string& chan,
							const torque_setpoint *setpoint)
			{
				model->GetJointController()->SetForce(rightHip3Joint->GetScopedName(), setpoint->tau1);
				model->GetJointController()->SetForce(rightHip2Joint->GetScopedName(), setpoint->tau2);
				model->GetJointController()->SetForce(rightHip1Joint->GetScopedName(), setpoint->tau3);
				model->GetJointController()->SetForce(rightKneeJoint->GetScopedName(), setpoint->tau4);
				model->GetJointController()->SetForce(rightAnkleJoint->GetScopedName(), setpoint->tau5);
			}
	};

  	/// \brief A plugin to control a Velodyne sensor.
	class BipedPlugin : public ModelPlugin
	{
		/// \brief Constructor
		public: BipedPlugin() {}

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.

		std::vector<std::string> split_string(std::string str, char delimiter) {
			std::vector<std::string> results;

			boost::split(results, str, [&delimiter](char c){return c == delimiter;});

			return results;
		}


		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			model = _model;

			leftHip3Joint = model->GetJoint("simplified_biped::left_hip_axis_3_hip_axis_2_joint");
			leftHip2Joint = model->GetJoint("simplified_biped::left_hip_axis_2_hip_axis_1_joint");
			leftHip1Joint = model->GetJoint("simplified_biped::left_hip_axis_1_upper_leg_joint");
			leftKneeJoint = model->GetJoint("simplified_biped::left_knee_lower_leg_joint");
			leftAnkleJoint = model->GetJoint("simplified_biped::left_ankle_foot_base_joint");

			rightHip3Joint = model->GetJoint("simplified_biped::right_hip_axis_3_hip_axis_2_joint");
			rightHip2Joint = model->GetJoint("simplified_biped::right_hip_axis_2_hip_axis_1_joint");
			rightHip1Joint = model->GetJoint("simplified_biped::right_hip_axis_1_upper_leg_joint");
			rightKneeJoint = model->GetJoint("simplified_biped::right_knee_lower_leg_joint");
			rightAnkleJoint = model->GetJoint("simplified_biped::right_ankle_foot_base_joint");

			double friction = 0.01;

			leftHip3Joint->SetParam("friction", 0, friction);
			leftKneeJoint->SetParam("friction", 0, friction);
			leftHip2Joint->SetParam("friction", 0, friction);
			leftHip1Joint->SetParam("friction", 0, friction);
			leftAnkleJoint->SetParam("friction", 0, friction);

			rightHip3Joint->SetParam("friction", 0, friction);
			rightKneeJoint->SetParam("friction", 0, friction);
			rightHip2Joint->SetParam("friction", 0, friction);
			rightHip1Joint->SetParam("friction", 0, friction);
			rightAnkleJoint->SetParam("friction", 0, friction);

			Handler handlerObject;

    		auto left_leg_subs = zcm_context.subscribe(left_leg_torque_setpoint_channel, &Handler::handleLeftLegTorqueSetpointMessage, &handlerObject);
    		auto right_leg_subs = zcm_context.subscribe(right_leg_torque_setpoint_channel, &Handler::handleRightLegTorqueSetpointMessage, &handlerObject);

			//leftLegStateThread = std::thread(std::bind(&BipedPlugin::PublishLeftLegState, this));	
			//rightLegStateThread = std::thread(std::bind(&BipedPlugin::PublishRightLegState, this));
			leftLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyLeftLegTorques, this));
		}

		public: void ApplyLeftLegTorques() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;

			long long iteration_counter = 0;

			int sockfd; 
			char buffer[udp_buffer_size]; 
			struct sockaddr_in     servaddr; 
		
			// Creating socket file descriptor 
			if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
				perror("socket creation failed"); 
				exit(EXIT_FAILURE); 
			} 
		
			memset(&servaddr, 0, sizeof(servaddr)); 
			
			// Filling server information 
			servaddr.sin_family = AF_INET; 
			servaddr.sin_port = htons(udp_port); 
			servaddr.sin_addr.s_addr = INADDR_ANY; 
			
			int n; 
			socklen_t len;
			
			//Initial message for connection to work properly.
			stringstream first_msg;
			first_msg << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
				<< leftAnkleJoint->Position() 
				<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" 
				<< leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);
			
			sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

			while(true) {
				start = high_resolution_clock::now();

				//std::cout << "Iteration count:" << iteration_counter << std::endl;

				stringstream s;

				s << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
					<< leftAnkleJoint->Position() 
					<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" 
					<< leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);

				
				sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
				// std::cout << "Sent out state." << std::endl;
				n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
				buffer[n] = '\0';

				//std::cout << "Received torque setpoint." << std::endl;

				string data(buffer);

				std::vector<std::string> torques = split_string(data, '|');

				if(static_cast<int>(torques.size()) >= 4) {

					double tau_1 = atof(torques[0].c_str());
					double tau_2 = atof(torques[1].c_str());
					double tau_3 = atof(torques[2].c_str());
					double tau_4 = atof(torques[3].c_str());
					double tau_5 = atof(torques[4].c_str());

					model->GetJointController()->SetForce(leftHip3Joint->GetScopedName(), tau_1);
					model->GetJointController()->SetForce(leftHip2Joint->GetScopedName(), tau_2);
					model->GetJointController()->SetForce(leftHip1Joint->GetScopedName(), tau_3);
					model->GetJointController()->SetForce(leftKneeJoint->GetScopedName(), tau_4);
					model->GetJointController()->SetForce(leftAnkleJoint->GetScopedName(), tau_5);

					std::cout << "Torque vector: " << tau_1 << "," << tau_2 << "," << tau_3 << "," << tau_4 << "," << tau_5 << std::endl;
				}

				iteration_counter++;

				end = high_resolution_clock::now();
				duration = duration_cast<microseconds>(end - start).count();

				std::cout << "Loop duration in uS:" << duration << std::endl;
				long long remainder = (torqueApplyingInterval - duration) * 1e+03;
				deadline.tv_nsec = remainder;
				deadline.tv_sec = 0;
				clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
			}
		}

		public: void PublishLeftLegState() {

			high_resolution_clock::time_point start = high_resolution_clock::now();
			high_resolution_clock::time_point end = high_resolution_clock::now();

			double duration = 0.0f;

			struct timespec deadline;

			int sockfd; 
			char buffer[udp_buffer_size];
			struct sockaddr_in servaddr, cliaddr;

			int n;

			socklen_t len;

			// Creating socket file descriptor 
			if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
				perror("socket creation failed"); 
				exit(EXIT_FAILURE); 
			} 
			
			memset(&servaddr, 0, sizeof(servaddr)); 
			memset(&cliaddr, 0, sizeof(cliaddr)); 
			
			// Filling server information 
			servaddr.sin_family = AF_INET; // IPv4 
			servaddr.sin_addr.s_addr = INADDR_ANY;
			servaddr.sin_port = htons(udp_port); 
			
			// Bind the socket with the server address 
			if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
			{ 
				perror("bind failed"); 
				exit(EXIT_FAILURE); 
			}

			while(true) {
				start = high_resolution_clock::now();

				stringstream s;

				s << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
				<< leftAnkleJoint->Position() 
				<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" 
				<< leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);

				sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				//std::cout << "Sent state, not my fault it doesn't work..." << std::endl;
				
				leg_state state;

				state.theta1 = leftHip3Joint->Position();
				state.theta2 = leftHip2Joint->Position();
				state.theta3 = leftHip1Joint->Position();
				state.theta4 = leftKneeJoint->Position();
				state.theta5 = leftAnkleJoint->Position();

				state.theta1_dot = leftHip3Joint->GetVelocity(0);
				state.theta2_dot = leftHip2Joint->GetVelocity(0);
				state.theta3_dot = leftHip1Joint->GetVelocity(0);
				state.theta4_dot = leftKneeJoint->GetVelocity(0);
				state.theta5_dot = leftAnkleJoint->GetVelocity(0);

				zcm_context.publish(left_leg_state_channel, &state);

				//std::cout << "Published left leg state." << std::endl;

				end = high_resolution_clock::now();

				duration = duration_cast<microseconds>(end - start).count();
				long long remainder = (statePublishingInterval - duration) * 1e+03;
				deadline.tv_nsec = remainder;
				deadline.tv_sec = 0;
				clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
			}
		}

		public: void PublishRightLegState() {

			high_resolution_clock::time_point start = high_resolution_clock::now();
			high_resolution_clock::time_point end = high_resolution_clock::now();

			double duration = 0.0f;

			struct timespec deadline;

			while(true) {

				start = high_resolution_clock::now();

				leg_state state;

				state.theta1 = rightHip3Joint->Position();
				state.theta2 = rightHip2Joint->Position();
				state.theta3 = rightHip1Joint->Position();
				state.theta4 = rightKneeJoint->Position();
				state.theta5 = rightAnkleJoint->Position();

				state.theta1_dot = rightHip3Joint->GetVelocity(0);
				state.theta2_dot = rightHip2Joint->GetVelocity(0);
				state.theta3_dot = rightHip1Joint->GetVelocity(0);
				state.theta4_dot = rightKneeJoint->GetVelocity(0);
				state.theta5_dot = rightAnkleJoint->GetVelocity(0);

				zcm_context.publish(right_leg_state_channel, &state);

				end = high_resolution_clock::now();
				duration = duration_cast<microseconds>(end - start).count();
				long long remainder = (statePublishingInterval - duration) * 1e+03;
				deadline.tv_nsec = remainder;
				deadline.tv_sec = 0;
				clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
			}
		}
	};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(BipedPlugin)
}
#endif

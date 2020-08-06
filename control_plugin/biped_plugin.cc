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
#include <Eigen_unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>

using Eigen::MatrixXd;

//#include "torque_setpoint.hpp"
//#include "leg_state.hpp"s

#include <unistd.h>

using namespace std;
using namespace std::chrono;


namespace gazebo
{
	gazebo::physics::ModelPtr model;

	std::thread leftLegStateThread;
	std::thread rightLegStateThread;
	std::thread leftLegTorqueThread;
	std::thread rightLegTorqueThread;

	std::thread disturbance_thread;
	std::thread mpc_force_thread;
	std::thread mpc_parse_thread;

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
	
	gazebo::physics::LinkPtr torso;

	ignition::math::Vector3d f_l(0, 0, 0); // Left foot force expressed in world frame
	ignition::math::Vector3d f_r(0, 0, 0); // Right foot force expressed in world frame
	
	ignition::math::Vector3d r_l(0, 0, 0); // Position where force is excerted expressed in CoM frame
	ignition::math::Vector3d r_r(0, 0, 0); // Position where force is excerted expressed in CoM frame


	static const double torqueApplyingInterval = 1000; // microseconds
	static const double statePublishingInterval = 1000; // microseconds
	static const double mpcInterval = (1/30.0) * 1000.0 * 1000.0; // microseconds, make sure this is the same as in Controller code!

	const char* left_leg_state_channel = "left_leg_state";
	const char* right_leg_state_channel = "right_leg_state";

	const char* left_leg_torque_setpoint_channel = ".left_leg_torque_setpoint";
	const char* right_leg_torque_setpoint_channel = ".right_leg_torque_setpoint";

	const int left_leg_port = 4200;
	const int right_leg_port = 4201;

	const int udp_buffer_size = 4096;

	const int udp_disturbance_port = 6768;

	const int udp_mpc_port = 4801;

	const bool legs_attached = false;

	// Pointer to the update event connection
	event::ConnectionPtr updateConnection;

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
			//updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BipedPlugin::OnUpdate, this));

			model = _model;
			torso = model->GetChildLink("simplified_biped::torso_connection");

			if(legs_attached) {
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
				double friction = 0.05; // Coulomb friction coefficient

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

				//leftLegStateThread = std::thread(std::bind(&BipedPlugin::PublishLeftLegState, this));	
				//rightLegStateThread = std::thread(std::bind(&BipedPlugin::PublishRightLegState, this));
				leftLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyLeftLegTorques, this));
				rightLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyRightLegTorques, this));
				disturbance_thread = std::thread(std::bind(&BipedPlugin::ApplyDisturbance, this));
			}
			mpc_force_thread = std::thread(std::bind(&BipedPlugin::ApplyMPCForces, this));
			mpc_parse_thread = std::thread(std::bind(&BipedPlugin::UpdateMPCForces, this));
		}

		public: void filter_value(double &val) {
			if(isnan(val) || isinf(val)) {
				val = 0;
			}
		}

		public: void UpdateMPCForces() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;
			
			int sockfd;
			char buffer[udp_buffer_size];
			struct sockaddr_in servaddr;
			
			// Creating socket file descriptor
			if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
				perror("socket creation failed");
				exit(EXIT_FAILURE);
			}
			
			memset(&servaddr, 0, sizeof(servaddr));
			
			// Filling server information
			servaddr.sin_family    = AF_INET; // IPv4
			servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
			servaddr.sin_port = htons(udp_mpc_port);

			int n;
		
			socklen_t len = sizeof(servaddr);  //len is value/result

			std::cout << "MPC Socket set up." << std::endl;
			
			stringstream first_msg;

			first_msg << torso->WorldPose().Rot().Euler().X() << "|" << torso->WorldPose().Rot().Euler().Y() << "|" << torso->WorldPose().Rot().Euler().Z() << "|" << torso->WorldPose().Pos().X() << "|" 
					<< torso->WorldPose().Pos().Y() << "|" << torso->WorldPose().Pos().Z() << "|" << torso->WorldAngularVel().X() << "|" << torso->WorldAngularVel().Y() << "|" 
					<< torso->WorldAngularVel().Z() << "|" << torso->WorldLinearVel().X() << "|" << torso->WorldLinearVel().Y() << "|" << torso->WorldLinearVel().Z() << "|-9.81";

			sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

			Eigen::Matrix<double, 4,4> H_body_world;
			Eigen::Matrix<double, 4,1> r_left_vector;
			Eigen::Matrix<double, 4,1> r_right_vector;
			Eigen::Matrix<double, 4,1> r_left_world_vector;
			Eigen::Matrix<double, 4,1> r_right_world_vector;

			ofstream data_file;
			data_file.open("../mpc_log.csv");
			data_file << "t,phi,theta,psi,pos_x,pos_y,pos_z,omega_x,omega_y,omega_z,vel_x,vel_y,vel_z,g,f_x_left,f_y_left,f_z_left,f_x_right,f_y_right,f_z_right,r_x_left,r_y_left,r_z_left,r_x_right,r_y_right,r_z_right,theta_delay_compensation,full_iteration_time,phi_delay_compensation" << std::endl; // Add header to csv file
			data_file.close();

			long long total_iterations = 0;

			while(true) {
				start = high_resolution_clock::now();

				stringstream s;

				// state is phi, theta, psi, p_x, p_y, p_z, omega_x, omega_y, omega_z, v_x, v_y, v_z, gravity constant
				double phi = torso->WorldPose().Rot().Roll();
				filter_value(phi);
				double theta = torso->WorldPose().Rot().Pitch();
				filter_value(theta);
				double psi = torso->WorldPose().Rot().Yaw();
				filter_value(psi);

				double pos_x = torso->WorldPose().Pos().X();
				filter_value(pos_x);
				double pos_y = torso->WorldPose().Pos().Y();
				filter_value(pos_y);
				double pos_z = torso->WorldPose().Pos().Z();
				filter_value(pos_z);

				double omega_x = torso->WorldAngularVel().X();
				filter_value(omega_x);
				double omega_y = torso->WorldAngularVel().Y();
				filter_value(omega_y);
				double omega_z = torso->WorldAngularVel().Z();
				filter_value(omega_z);

				double vel_x = torso->WorldLinearVel().X();
				filter_value(vel_x);
				double vel_y = torso->WorldLinearVel().Y();
				filter_value(vel_y);
				double vel_z = torso->WorldLinearVel().Z();
				filter_value(vel_z);

				//std::cout << "omega_x:" << torso->WorldAngularVel().X() << ",omega_y:" << torso->WorldAngularVel().Y() << ",omega_z:" << torso->WorldAngularVel().Z() 
				//<< ",vel_x:" << torso->WorldLinearVel().X() << ",vel_y:" << torso->WorldLinearVel().Y() << ",vel_z:" << torso->WorldLinearVel().Z() << std::endl;

				s << phi << "|" << theta << "|" << psi << "|" << pos_x << "|" 
					<< pos_y << "|" << pos_z << "|" << omega_x << "|" << omega_y << "|" 
					<< omega_z << "|" << vel_x << "|" << vel_y << "|" << vel_z << "|-9.81";

				std::cout << "UDP message:" << s.str() << std::endl;

				sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, (struct sockaddr *) &servaddr, &len); 
				buffer[n] = '\0';

				string data_str(buffer);

				std::vector<std::string> message_split = split_string(data_str, '|');

				if(static_cast<int>(message_split.size()) >= 11) {
					ignition::math::Vector3d f_l_temp(atof(message_split[0].c_str()), atof(message_split[1].c_str()), atof(message_split[2].c_str()));
					ignition::math::Vector3d f_r_temp(atof(message_split[3].c_str()), atof(message_split[4].c_str()), atof(message_split[5].c_str()));

					f_l = f_l_temp;
					f_r = f_r_temp;

					double phi = torso->WorldPose().Rot().Roll();
					filter_value(phi);
					double theta = torso->WorldPose().Rot().Pitch();
					filter_value(theta);
					double psi = torso->WorldPose().Rot().Yaw();
					filter_value(psi);

					double pos_x = torso->WorldPose().Pos().X();
					filter_value(pos_x);
					double pos_y = torso->WorldPose().Pos().Y();
					filter_value(pos_y);
					double pos_z = torso->WorldPose().Pos().Z();
					filter_value(pos_z);

					double omega_x = torso->WorldAngularVel().X();
					filter_value(omega_x);
					double omega_y = torso->WorldAngularVel().Y();
					filter_value(omega_y);
					double omega_z = torso->WorldAngularVel().Z();
					filter_value(omega_z);

					double vel_x = torso->WorldLinearVel().X();
					filter_value(vel_x);
					double vel_y = torso->WorldLinearVel().Y();
					filter_value(vel_y);
					double vel_z = torso->WorldLinearVel().Z();
					filter_value(vel_z);

					double r_x_left = atof(message_split[6].c_str());
					double r_y_left = atof(message_split[7].c_str());
					double r_z_left = atof(message_split[8].c_str());

					r_left_vector << r_x_left, r_y_left, r_z_left, 1; // 0 to match transformation matrix dimensions, otherwise obsolete and will not be used
					
					double r_x_right = atof(message_split[9].c_str());
					double r_y_right = atof(message_split[10].c_str());
					double r_z_right = atof(message_split[11].c_str());

					r_right_vector << r_x_right, r_y_right, r_z_right, 1; // 0 to match transformation matrix dimensions, otherwise obsolete and will not be used

					H_body_world << cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi), sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi), pos_x,
									sin(psi)*cos(theta), sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi), -sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi), pos_y,
									-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta), pos_z,
									0, 0, 0, 1;

					r_left_world_vector = H_body_world * r_left_vector;
					r_right_world_vector = H_body_world * r_right_vector;
					
					double r_x_left_world = r_x_left + sin(theta) * r_z_left;
					double r_x_right_world = r_x_right + sin(theta) * r_z_right;

					double r_y_left_world = r_y_left - r_z_left * sin(phi);
					double r_y_right_world = r_y_right - r_z_right * sin(phi);

					double r_z_left_world = 0;
					double r_z_right_world = 0;

					//std::cout << "r_x_left_world: " << r_x_left_world << ",r_x_right_world: " << r_x_right_world << ",r_y_left_world: " << r_y_left_world << ",r_y_right_world: " << r_y_right_world << std::endl;

					//ignition::math::Vector3d r_l_temp(r_x_left_world + pos_x, r_y_left_world + pos_y, r_z_left_world);
					//ignition::math::Vector3d r_r_temp(r_x_right_world + pos_x, r_y_right_world + pos_y, r_z_right_world);

					// ignition::math::Vector3d r_l_temp(r_left_world_vector(0), r_left_world_vector(1), r_left_world_vector(2));
					// ignition::math::Vector3d r_r_temp(r_right_world_vector(0), r_right_world_vector(1), r_right_world_vector(2));

					ignition::math::Vector3d r_l_temp(r_x_left + pos_x, r_y_left + pos_y, r_z_left + pos_z);
					ignition::math::Vector3d r_r_temp(r_x_right + pos_x, r_y_right + pos_y, r_z_right + pos_z);
					
					r_l = r_l_temp;
					r_r = r_r_temp;

					ofstream data_file;
					data_file.open("../mpc_log.csv", ios::app); // Open csv file in append mode
					data_file << total_iterations * (mpcInterval / 1e+6) << "," << phi << "," << theta << "," << psi << "," 
								<< pos_x << "," << pos_y << "," << pos_z << ","
								<< omega_x << "," << omega_y << "," << omega_z << ","
								<< vel_x << "," << vel_y << "," << vel_z << "," << -9.81 << ","
								<< f_l[0] << "," << f_l[1] << "," << f_l[2] << ","
								<< f_r[0] << "," << f_r[1] << "," << f_r[2] << ","
								<< r_l[0] << "," << r_l[1] << "," << r_l[2] << "," 
								<< r_r[0] << "," << r_r[1] << "," << r_r[2] << ","
								<< atof(message_split[12].c_str()) << "," << atof(message_split[13].c_str()) << "," << atof(message_split[14].c_str())
								<< std::endl;
					data_file.close(); // Close csv file again. This way thread abort should (almost) never leave file open.

					//std::cout << "r_left_world: " << r_l << "\tr_right_world: " << r_r << std::endl;
				}

				total_iterations++;
				
				end = high_resolution_clock::now();

				duration = duration_cast<microseconds>(end - start).count();
				long long remainder = (mpcInterval - duration) * 1e+3;
				// deadline.tv_nsec = remainder;
				// deadline.tv_sec = 0;
				// clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
			}
		}

		public: void ApplyMPCForces() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;

			while(true) {
				start = high_resolution_clock::now();

				//std::cout << "f_l: " << f_l << std::endl;
				//std::cout << "f_r: " << f_r << std::endl;
				//std::cout << "r_l_world: " << r_l << std::endl;
				//std::cout << "r_r_world: " << r_r << std::endl;

				torso->AddForceAtWorldPosition(f_l, r_l);
				torso->AddForceAtWorldPosition(f_r, r_r);

				end = high_resolution_clock::now();
				duration = duration_cast<microseconds>(end - start).count();
				//std::cout << "MPC force excertion loop duration in µS:" << duration << std::endl;
				long long remainder = (torqueApplyingInterval - duration) * 1e+03; // nanoseconds
				deadline.tv_nsec = remainder;
				deadline.tv_sec = 0;
				clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
			}
		}

		public: void OnUpdate() {
			
		}

		public: void ApplyDisturbance() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;
			
			int sockfd; 
			char buffer[udp_buffer_size];
			struct sockaddr_in servaddr, cliaddr; 
			
			// Creating socket file descriptor 
			if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
				perror("socket creation failed"); 
				exit(EXIT_FAILURE); 
			} 
			
			memset(&servaddr, 0, sizeof(servaddr)); 
			memset(&cliaddr, 0, sizeof(cliaddr));
			
			// Filling server information
			servaddr.sin_family    = AF_INET; // IPv4
			servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
			servaddr.sin_port = htons(udp_disturbance_port);
			
			// Bind the socket with the server address 
			if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
					sizeof(servaddr)) < 0 )
			{
				perror("bind failed");
				exit(EXIT_FAILURE);
			}
			
			int n; 
		
			socklen_t len = sizeof(cliaddr);  //len is value/result 

			std::cout << "Disturbance Socket set up." << std::endl;

			while(true) {

				n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); 
				buffer[n] = '\0'; 

				std::cout << "Received message, processing..." << std::endl;

				string data(buffer);

				std::vector<std::string> message_split = split_string(data, '|');

				double dt = 1000; // microseconds

				if(static_cast<int>(message_split.size()) >= 2) {
					
					std::vector<std::string> force_components = split_string(message_split[1], ',');

					ignition::math::Vector3d force(atof(force_components[0].c_str()), atof(force_components[1].c_str()), atof(force_components[2].c_str()));

					double disturbance_duration = atof(message_split[2].c_str());

					auto disturbance_link = model->GetLink("simplified_biped::" + message_split[0]);

					std::cout << "Finished parsing, entering loop. Selected Link name: " << disturbance_link << ", force: " << force.Length() << ", Duration: " << disturbance_duration << std::endl;

					if(disturbance_link != NULL) {

						int counter = 0;

						for(int i = 0; i < (dt/1000) * disturbance_duration * 1000 /*multiply by 1000 to get back to ms*/; ++i) {
							
							start = high_resolution_clock::now();
							disturbance_link->AddLinkForce(force);
							end = high_resolution_clock::now();
							duration = duration_cast<microseconds> (end - start).count();
							std::cout << "Applied disturbance for one iteration. Link name: " << disturbance_link << ", force: " << force.Length() << ", Duration: " << disturbance_duration << std::endl;
							counter++;
							long long remainder = (dt - duration) * 1e+03;
							deadline.tv_nsec = remainder;
							deadline.tv_sec = 0;
							clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
						}

						std::cout << "Applying disturbance, counter: " << counter << std::endl;
					}
					else {
							std::cerr << "Selected link lead to null pointer..." << std::endl;
					}
				}
			}
		}

		public: void ApplyLeftLegTorques() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;

			long long iteration_counter = 0;

			int sockfd; 
			char buffer[udp_buffer_size]; 
			struct sockaddr_in servaddr; 
		
			// Creating socket file descriptor 
			if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
				perror("socket creation failed"); 
				exit(EXIT_FAILURE); 
			} 
		
			memset(&servaddr, 0, sizeof(servaddr)); 
			
			// Filling server information 
			servaddr.sin_family = AF_INET; 
			servaddr.sin_port = htons(left_leg_port); 
			servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
			
			int n; 
			socklen_t len = sizeof(servaddr);

			if (legs_attached) {
			
				//Initial message for connection to work properly.
				stringstream first_msg;
				first_msg << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
					<< leftAnkleJoint->Position() 
					<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" 
					<< leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);
				
				sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				while(true) {
					start = high_resolution_clock::now();

					stringstream s;
					s << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
						<< leftAnkleJoint->Position() 
						<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" 
						<< leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);

					
					sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
					
					n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, (struct sockaddr *) &servaddr, &len); 
					buffer[n] = '\0';

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

					std::cout << "Left leg torque loop duration in µS:" << duration << std::endl;
					long long remainder = (torqueApplyingInterval - duration) * 1e+03;
					deadline.tv_nsec = remainder;
					deadline.tv_sec = 0;
					clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
				}
			}
		}

		public: void ApplyRightLegTorques() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;

			long long iteration_counter = 0;

			int sockfd; 
			char buffer[udp_buffer_size]; 
			struct sockaddr_in servaddr; 
		
			// Creating socket file descriptor 
			if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
				perror("socket creation failed"); 
				exit(EXIT_FAILURE); 
			} 
		
			memset(&servaddr, 0, sizeof(servaddr)); 
			
			// Filling server information 
			servaddr.sin_family = AF_INET; 
			servaddr.sin_port = htons(right_leg_port); 
			servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
			
			int n; 
			socklen_t len = sizeof(servaddr);

			if(legs_attached) {
			
				//Initial message for connection to work properly.
				stringstream first_msg;
				first_msg << rightHip3Joint->Position() << "|" << rightHip2Joint->Position() << "|" << rightHip1Joint->Position() << "|" << rightKneeJoint->Position() << "|" 
					<< rightAnkleJoint->Position() 
					<< "|" << rightHip3Joint->GetVelocity(0) << "|" << rightHip2Joint->GetVelocity(0) << "|" << rightHip1Joint->GetVelocity(0) << "|" 
					<< rightKneeJoint->GetVelocity(0) << "|" << rightAnkleJoint->GetVelocity(0);
				
				sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				while(true) {
					start = high_resolution_clock::now();

					stringstream s;
					s << rightHip3Joint->Position() << "|" << rightHip2Joint->Position() << "|" << rightHip1Joint->Position() << "|" << rightKneeJoint->Position() << "|" 
						<< rightAnkleJoint->Position() 
						<< "|" << rightHip3Joint->GetVelocity(0) << "|" << rightHip2Joint->GetVelocity(0) << "|" << rightHip1Joint->GetVelocity(0) << "|" 
						<< rightKneeJoint->GetVelocity(0) << "|" << rightAnkleJoint->GetVelocity(0);

					
					sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
					n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, (struct sockaddr *) &servaddr, &len); 
					buffer[n] = '\0';

					std::cout << "Received left leg torque setpoint." << std::endl;

					string data(buffer);

					std::vector<std::string> torques = split_string(data, '|');

					if(static_cast<int>(torques.size()) >= 4) {

						double tau_1 = atof(torques[0].c_str());
						double tau_2 = atof(torques[1].c_str());
						double tau_3 = atof(torques[2].c_str());
						double tau_4 = atof(torques[3].c_str());
						double tau_5 = atof(torques[4].c_str());

						model->GetJointController()->SetForce(rightHip3Joint->GetScopedName(), tau_1);
						model->GetJointController()->SetForce(rightHip2Joint->GetScopedName(), tau_2);
						model->GetJointController()->SetForce(rightHip1Joint->GetScopedName(), tau_3);
						model->GetJointController()->SetForce(rightKneeJoint->GetScopedName(), tau_4);
						model->GetJointController()->SetForce(rightAnkleJoint->GetScopedName(), tau_5);

						std::cout << "Torque vector: " << tau_1 << "," << tau_2 << "," << tau_3 << "," << tau_4 << "," << tau_5 << std::endl;
					}

					iteration_counter++;

					end = high_resolution_clock::now();
					duration = duration_cast<microseconds>(end - start).count();

					std::cout << "Right leg torque loop duration in µS:" << duration << std::endl;
					long long remainder = (torqueApplyingInterval - duration) * 1e+03;
					deadline.tv_nsec = remainder;
					deadline.tv_sec = 0;
					clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
				}
			}
		}
	};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(BipedPlugin)
}
#endif

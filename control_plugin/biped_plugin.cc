#ifndef _BIPED_PLUGIN_HH_
#define _BIPED_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo_client.hh>

#include <string>
#include <random>
#include <chrono>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <mutex>
#include <unistd.h>

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <math.h>

using Eigen::MatrixXd;

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

	std::thread sim_state_thread;
	std::mutex sim_state_mutex;

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
	gazebo::physics::LinkPtr left_foot_base;
	gazebo::physics::LinkPtr right_foot_base;

	ignition::math::Vector3d f_l(0, 0, 0); // Left foot force expressed in world frame
	ignition::math::Vector3d f_r(0, 0, 0); // Right foot force expressed in world frame
	
	ignition::math::Vector3d r_l(0, 0, 0); // Position where force is excerted expressed in CoM frame
	ignition::math::Vector3d r_r(0, 0, 0); // Position where force is excerted expressed in CoM frame

	static const double torqueApplyingInterval = 960; // microseconds, some margin to account for comms delays etc.
	static const double statePublishingInterval = 960; // microseconds
	static const double mpcInterval = (1/30.0) * 1000.0 * 1000.0; // microseconds, make sure this is the same as in Controller code!

	const int left_leg_port = 4200;
	const int right_leg_port = 4201;
	const int sim_state_port = 4202;

	const int udp_buffer_size = 4096;

	const int udp_disturbance_port = 6768;

	const int udp_mpc_port = 4801;

	static const int n = 13, m = 6;

	const bool legs_attached = true;
	const bool apply_torques = true;
	const bool apply_forces = false;
	const bool print_torque_vectors = false;

	double prev_psi = 0;

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

		public: void print_threadsafe(std::string str, std::string sender) {
			std::string prepared_string = "\033[1;36m[From '" + sender + "']:\033[0m \033[1;33m'" + str + "'\033[0m\n";
			std::cout << prepared_string;
		}

		public: void filter_value(double &val) {
			if(isnan(val) || isinf(val)) {
				val = 0;
			}
		}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BipedPlugin::OnUpdate, this));

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

				left_foot_base = model->GetChildLink("simplified_biped::left_foot_base");
				right_foot_base = model->GetChildLink("simplified_biped::right_foot_base");

				leftHip3Joint->SetPosition(0, 0);
				leftHip2Joint->SetPosition(0, 0);
				leftHip1Joint->SetPosition(0, -0.4);
				leftKneeJoint->SetPosition(0, 0.85);
				leftAnkleJoint->SetPosition(0, -0.45);

				rightHip3Joint->SetPosition(0, 0);
				rightHip2Joint->SetPosition(0, 0);
				rightHip1Joint->SetPosition(0, -0.4);
				rightKneeJoint->SetPosition(0, 0.85);
				rightAnkleJoint->SetPosition(0, -0.45);

				model->GetJointController()->SetPositionPID(leftHip3Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(leftHip2Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(leftHip1Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(leftKneeJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(leftAnkleJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));

				model->GetJointController()->SetVelocityPID(leftHip3Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(leftHip2Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(leftHip1Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(leftKneeJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(leftAnkleJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));

				model->GetJointController()->SetPositionPID(rightHip3Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(rightHip2Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(rightHip1Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(rightKneeJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetPositionPID(rightAnkleJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));

				model->GetJointController()->SetVelocityPID(rightHip3Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(rightHip2Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(rightHip1Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(rightKneeJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));
				model->GetJointController()->SetVelocityPID(rightAnkleJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));

				// model->SetJointPosition("simplified_biped::left_hip_axis_3_hip_axis_2_joint", 1);

				double friction = 0.05; // Coulomb friction coefficient

				// leftHip3Joint->SetParam("friction", 0, friction);
				// leftKneeJoint->SetParam("friction", 0, friction);
				// leftHip2Joint->SetParam("friction", 0, friction);
				// leftHip1Joint->SetParam("friction", 0, friction);
				// leftAnkleJoint->SetParam("friction", 0, friction);

				// rightHip3Joint->SetParam("friction", 0, friction);
				// rightKneeJoint->SetParam("friction", 0, friction);
				// rightHip2Joint->SetParam("friction", 0, friction);
				// rightHip1Joint->SetParam("friction", 0, friction);
				// rightAnkleJoint->SetParam("friction", 0, friction);

				//leftLegStateThread = std::thread(std::bind(&BipedPlugin::PublishLeftLegState, this));	
				//rightLegStateThread = std::thread(std::bind(&BipedPlugin::PublishRightLegState, this));
				leftLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyLeftLegTorques, this));
				rightLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyRightLegTorques, this));
				disturbance_thread = std::thread(std::bind(&BipedPlugin::ApplyDisturbance, this));
			}
			mpc_force_thread = std::thread(std::bind(&BipedPlugin::ApplyMPCForces, this));
			mpc_parse_thread = std::thread(std::bind(&BipedPlugin::UpdateMPCForces, this));
		}

		public: double offset = 0;

		public: Eigen::Matrix<double, n, 1> get_CoMState() {
			// state is phi, theta, psi, p_x, p_y, p_z, omega_x, omega_y, omega_z, v_x, v_y, v_z, gravity constant
			double phi = torso->WorldPose().Rot().Roll();
			filter_value(phi);
			double theta = torso->WorldPose().Rot().Pitch();
			filter_value(theta);
			double psi = torso->WorldPose().Rot().Yaw();
			filter_value(psi);

			// std::cout << "signbit psi=" << signbit(psi) << ",signbit prev_psi=" << signbit(prev_psi) << ",psi=" << psi << ",prev_psi=" << prev_psi << "\n";

			if(signbit(psi) != signbit(prev_psi) && (prev_psi > M_PI_2 || prev_psi < -M_PI_2)) {
				offset += (signbit(psi) ? 2 * M_PI : -2 * M_PI);
				// std::cout << "Offset applied, psi=" << psi << "\n";
			}

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

			double g = -9.81;

			prev_psi = psi;
			psi += offset;

			return (Eigen::Matrix<double, n, 1>() << phi, theta, psi, pos_x, pos_y, pos_z, omega_x, omega_y, omega_z, vel_x, vel_y, vel_z, g).finished();
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

			int msg_length;
		
			socklen_t len = sizeof(servaddr);  // len is value/result

			print_threadsafe("MPC Socket set up.", "mpc_update_thread");
			
			stringstream first_msg;

			Eigen::Matrix<double, n, 1> state = get_CoMState();

			for(int i = 0; i < n; i++) {
				first_msg << state(i, 0);

				if(i != n - 1) { // Don't append separator to end
					first_msg << "|";
				}
			}

			sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

			Eigen::Matrix<double, 4,4> H_body_world;
			Eigen::Matrix<double, 4,1> r_left_vector;
			Eigen::Matrix<double, 4,1> r_right_vector;
			Eigen::Matrix<double, 4,1> r_left_world_vector;
			Eigen::Matrix<double, 4,1> r_right_world_vector;

			ofstream data_file;
			data_file.open("../mpc_log.csv");
			data_file << "t_sim,phi,theta,psi,pos_x,pos_y,pos_z,omega_x,omega_y,omega_z,vel_x,vel_y,vel_z,g,f_x_left,f_y_left,f_z_left,f_x_right,f_y_right,f_z_right,r_x_left,r_y_left,r_z_left,r_x_right,r_y_right,r_z_right,theta_delay_compensation,full_iteration_time,phi_delay_compensation,prev_logging_time" << std::endl; // Add header to csv file
			data_file.close();

			long long total_iterations = 0;

			struct timeval tv;
			tv.tv_sec = 0; // Initially set to very high value to wait for first message because it takes some time to start up sim.
			tv.tv_usec = 100000;
			setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

			while(true) {
				start = high_resolution_clock::now();
				stringstream s;

				state = get_CoMState();

				for(int i = 0; i < n; i++) {
					s << state(i, 0);

					if(i != n - 1) { // Don't append seperator to end
						s << "|";
					}
				}

				sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, (struct sockaddr *) &servaddr, &len); 
				buffer[msg_length] = '\0';

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
					
					double r_x_right = atof(message_split[9].c_str());
					double r_y_right = atof(message_split[10].c_str());
					double r_z_right = atof(message_split[11].c_str());

					ignition::math::Vector3d r_l_temp(r_x_left + pos_x, r_y_left + pos_y, r_z_left + pos_z);
					ignition::math::Vector3d r_r_temp(r_x_right + pos_x, r_y_right + pos_y, r_z_right + pos_z);
					
					// ignition::math::Vector3d r_l_temp(r_x_left, r_y_left, r_z_left);
					// ignition::math::Vector3d r_r_temp(r_x_right, r_y_right, r_z_right);

					r_l = r_l_temp;
					r_r = r_r_temp;
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

			long long total_iterations = 0;

			while(true) {
				start = high_resolution_clock::now();

				//std::cout << "f_l: " << f_l << std::endl;
				//std::cout << "f_r: " << f_r << std::endl;
				//std::cout << "r_l_world: " << r_l << std::endl;
				//std::cout << "r_r_world: " << r_r << std::endl;
				if(apply_forces /*&& (total_iterations * (1/1000.0)) > 0.5*/) {
					torso->AddForceAtWorldPosition(f_l, r_l);
					torso->AddForceAtWorldPosition(f_r, r_r);
				}
				
				total_iterations++;

				end = high_resolution_clock::now();
				duration = duration_cast<microseconds>(end - start).count();
				//std::cout << "MPC force excertion loop duration in µS:" << duration << std::endl;
				long long remainder = (torqueApplyingInterval - duration) * 1e+03; // nanoseconds
				deadline.tv_nsec = remainder;
				deadline.tv_sec = 0;
				clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
			}
		}

		public: void ApplyDisturbance() {
			auto start = high_resolution_clock::now();
			auto end = high_resolution_clock::now();

			double duration;
			struct timespec deadline;
			
			int sockfd, portno, n;
			struct sockaddr_in serv_addr;
			struct hostent *server;

			const int buffer_size = 128;

			char buffer[buffer_size];
			portno = 6969;
			sockfd = socket(AF_INET, SOCK_STREAM, 0);
			if (sockfd < 0) 
				std::cerr << "Error opening socket.\n";
			
			server = gethostbyname("terminator.loukordos.eu");
			if (server == NULL) {
				fprintf(stderr,"ERROR, no such host\n");
				exit(0);
			}

			bzero((char *) &serv_addr, sizeof(serv_addr));
			serv_addr.sin_family = AF_INET;
			bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
			serv_addr.sin_port = htons(portno);

			if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
				std::cerr << "Error connecting to server.\n";

			stringstream temp;
			temp << "Disturbance Socket set up.";
			print_threadsafe(temp.str(), "disturbance_thread");

			while(true) {
				bzero(buffer, buffer_size);
				n = read(sockfd, buffer, buffer_size - 1);
				if (n < 0) {
					std::cout << "Error while reading from socket.\n";
				}
				std::string data(buffer);

				print_threadsafe("Received message, processing...", "disturbance_thread");

				std::vector<std::string> message_split = split_string(data, '|');

				double dt = 250; // microseconds

				if(static_cast<int>(message_split.size()) >= 2) {
					
					std::vector<std::string> force_components = split_string(message_split[1], ',');

					ignition::math::Vector3d force(atof(force_components[0].c_str()), atof(force_components[1].c_str()), atof(force_components[2].c_str()));
					
					double disturbance_duration = atof(message_split[2].c_str());

					stringstream temp;
					temp << "Trying to find link simplified_biped::" << message_split[0] << std::endl;

					auto disturbance_link = model->GetLink("simplified_biped::" + message_split[0]);

					temp.str("");
					temp << "Finished parsing, entering loop. Selected Link name: " << disturbance_link << ", force: " << force.Length() << ", Duration: " << disturbance_duration;
					print_threadsafe(temp.str(), "disturbance_thread");

					if(disturbance_link != NULL) {
						
						int counter = 0;

						for(int i = 0; i < (1000/dt) * disturbance_duration * 1000 /*multiply by 1000 to get back to ms*/; ++i) {
							
							start = high_resolution_clock::now();
							disturbance_link->SetForce(force);

							// stringstream temp;
							// temp << "Applied disturbance for one iteration. Link name: " << disturbance_link << ", Force Mag: " << force.Length() << ", Duration: " << disturbance_duration;
							// print_threadsafe(temp.str(), "disturbance_thread");

							end = high_resolution_clock::now();
							duration = duration_cast<microseconds> (end - start).count();
							
							counter++;
							long long remainder = (dt - duration) * 1e+03;
							deadline.tv_nsec = remainder;
							deadline.tv_sec = 0;
							clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
						}

						stringstream temp;
						temp << "Applying disturbance, counter: " << counter;
						print_threadsafe(temp.str(), "disturbance_thread");
					}
					else {
						print_threadsafe("Selected link lead to null pointer...", "disturbance_thread");
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
			
			int msg_length;

			socklen_t len = sizeof(servaddr);

			long prev_iteration_timestamp = -1; // Iteration timestamp sent with every message so that same message is not applied twice.

			if (legs_attached) {
			
				//Initial message for connection to work properly.
				stringstream first_msg;
				first_msg << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
					<< leftAnkleJoint->Position() 
					<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) 
					<< "|" << leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);

				first_msg << "|";

				Eigen::Matrix<double, n, 1> state = get_CoMState();

				for(int i = 0; i < n; i++) {
					first_msg << state(i, 0);

					if(i != n - 1) { // Don't append seperator to end
						first_msg << "|";
					}
				}

				first_msg << "|" << left_foot_base->WorldPose().Pos().X() << "|" << left_foot_base->WorldPose().Pos().Y() << "|" << left_foot_base->WorldPose().Pos().Z();
				
				sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				struct timeval tv;
				tv.tv_sec = 0; // Initially set to very high value to wait for first message because it takes some time to start up sim.
				tv.tv_usec = 100000;
				setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

				while(true) {
					start = high_resolution_clock::now();

					stringstream s;
					s << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" 
						<< leftAnkleJoint->Position() 
						<< "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" 
						<< leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);

					s << "|";

					Eigen::Matrix<double, n, 1> state = get_CoMState();

					for(int i = 0; i < n; i++) {
						s << state(i, 0);

						if(i != n - 1) { // Don't append seperator to end
							s << "|";
						}
					}

					s << "|" << left_foot_base->WorldPose().Pos().X() << "|" << left_foot_base->WorldPose().Pos().Y() << "|" << left_foot_base->WorldPose().Pos().Z();
					
					sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
					
					msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, (struct sockaddr *) &servaddr, &len); 
					buffer[msg_length] = '\0';

					string data(buffer);

					std::vector<std::string> torques = split_string(data, '|');

					if(static_cast<int>(torques.size()) >= 5) {

						double tau_1 = atof(torques[0].c_str());
						double tau_2 = atof(torques[1].c_str());
						double tau_3 = atof(torques[2].c_str());
						double tau_4 = atof(torques[3].c_str());
						double tau_5 = atof(torques[4].c_str());

						if(apply_torques) {
							if(atoi(torques[5].c_str()) != prev_iteration_timestamp) {
								model->GetJointController()->SetForce(leftHip3Joint->GetScopedName(), tau_1);
								model->GetJointController()->SetForce(leftHip2Joint->GetScopedName(), tau_2);
								model->GetJointController()->SetForce(leftHip1Joint->GetScopedName(), tau_3);
								model->GetJointController()->SetForce(leftKneeJoint->GetScopedName(), tau_4);
								model->GetJointController()->SetForce(leftAnkleJoint->GetScopedName(), tau_5);
							}
							else {
								print_threadsafe("Got outdated torque setpoint.", "left_leg_torque_thread");
							}
						}

						prev_iteration_timestamp = atoi(torques[5].c_str());

						if(print_torque_vectors) {
							stringstream temp;
							temp << "Torque vector: " << tau_1 << "," << tau_2 << "," << tau_3 << "," << tau_4 << "," << tau_5;
							print_threadsafe(temp.str(), "left_leg_torque_thread");
						}
					}

					iteration_counter++;

					end = high_resolution_clock::now();
					duration = duration_cast<microseconds>(end - start).count();
					
					// stringstream temp;
					// temp << "Left leg torque loop duration in µS:" << duration;
					// print_threadsafe(temp.str(), "left_leg_torque_thread");

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
			
			int msg_length;

			socklen_t len = sizeof(servaddr);

			long prev_iteration_timestamp = -1; // Iteration timestamp sent with every message so that same message is not applied twice.

			if(legs_attached) {
			
				//Initial message for connection to work properly.
				stringstream first_msg;
				first_msg << rightHip3Joint->Position() << "|" << rightHip2Joint->Position() << "|" << rightHip1Joint->Position() << "|" << rightKneeJoint->Position() << "|" 
					<< rightAnkleJoint->Position() 
					<< "|" << rightHip3Joint->GetVelocity(0) << "|" << rightHip2Joint->GetVelocity(0) << "|" << rightHip1Joint->GetVelocity(0) 
					<< "|" << rightKneeJoint->GetVelocity(0) << "|" << rightAnkleJoint->GetVelocity(0);
				
				first_msg << "|";

				Eigen::Matrix<double, n, 1> state = get_CoMState();

				for(int i = 0; i < n; i++) {
					first_msg << state(i, 0);

					if(i != n - 1) { // Don't append seperator to end
						first_msg << "|";
					}
				}

				// std::cout << right_foot_base->WorldPose().Pos().X() << std::endl;

				first_msg << "|" << right_foot_base->WorldPose().Pos().X() << "|" << right_foot_base->WorldPose().Pos().Y() << "|" << right_foot_base->WorldPose().Pos().Z();

				sendto(sockfd, (const char *)first_msg.str().c_str(), strlen(first_msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

				struct timeval tv;
				tv.tv_sec = 0; // Initially set to very high value to wait for first message because it takes some time to start up sim.
				tv.tv_usec = 100000;
				setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

				while(true) {
					start = high_resolution_clock::now();

					stringstream s;
					s << rightHip3Joint->Position() << "|" << rightHip2Joint->Position() << "|" << rightHip1Joint->Position() << "|" << rightKneeJoint->Position() << "|" 
						<< rightAnkleJoint->Position() 
						<< "|" << rightHip3Joint->GetVelocity(0) << "|" << rightHip2Joint->GetVelocity(0) << "|" << rightHip1Joint->GetVelocity(0) << "|" 
						<< rightKneeJoint->GetVelocity(0) << "|" << rightAnkleJoint->GetVelocity(0);

					s << "|";

					Eigen::Matrix<double, n, 1> state = get_CoMState();

					for(int i = 0; i < n; i++) {
						s << state(i, 0);

						if(i != n - 1) { // Don't append seperator to end
							s << "|";
						}
					}

					s << "|" << right_foot_base->WorldPose().Pos().X() << "|" << right_foot_base->WorldPose().Pos().Y() << "|" << right_foot_base->WorldPose().Pos().Z();

					sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
					msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, (struct sockaddr *) &servaddr, &len); 
					buffer[msg_length] = '\0';

					// print_threadsafe("Received left leg torque setpoint.", "right_leg_torque_thread");

					string data(buffer);

					std::vector<std::string> torques = split_string(data, '|');

					if(static_cast<int>(torques.size()) >= 5) {

						double tau_1 = atof(torques[0].c_str());
						double tau_2 = atof(torques[1].c_str());
						double tau_3 = atof(torques[2].c_str());
						double tau_4 = atof(torques[3].c_str());
						double tau_5 = atof(torques[4].c_str());

						if(apply_torques) {
							if(atoi(torques[5].c_str()) != prev_iteration_timestamp) {
								model->GetJointController()->SetForce(rightHip3Joint->GetScopedName(), tau_1);
								model->GetJointController()->SetForce(rightHip2Joint->GetScopedName(), tau_2);
								model->GetJointController()->SetForce(rightHip1Joint->GetScopedName(), tau_3);
								model->GetJointController()->SetForce(rightKneeJoint->GetScopedName(), tau_4);
								model->GetJointController()->SetForce(rightAnkleJoint->GetScopedName(), tau_5);
							}
							else {
								stringstream temp;
								temp << "Got outdated torque setpoint.";
								print_threadsafe(temp.str(), "right_leg_torque_thread");
							}
							
						}

						prev_iteration_timestamp = atoi(torques[5].c_str());

						if(print_torque_vectors) {
							stringstream temp;
							temp << "Torque vector: " << tau_1 << "," << tau_2 << "," << tau_3 << "," << tau_4 << "," << tau_5;
							print_threadsafe(temp.str(), "right_leg_torque_thread");
						}
					}

					iteration_counter++;

					end = high_resolution_clock::now();
					duration = duration_cast<microseconds>(end - start).count();

					// stringstream temp;
					// temp << "Right leg torque loop duration in µS:" << duration;
					// print_threadsafe(temp.str(), "right_leg_torque_thread");

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

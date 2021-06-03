#ifndef _GAZEBO_WORLD_STATE_PLUGIN_HH_
#define _GAZEBO_WORLD_STATE_PLUGIN_HH_

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <string>
#include <random>

#include <chrono>

#include <ctime>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <unistd.h>

#include <mutex>
#include <thread>
#include <functional>

using namespace std::chrono;

namespace gazebo
{
    class SimStatePlugin : public WorldPlugin
    {
        public: SimStatePlugin() {}

        public: std::thread sim_state_update_thread;

        public: static const int udp_buffer_size = 4096;
        public: static const int sim_state_port = 4202;

        public: void update_sim_state() {
            // High resolution clocks used for measuring execution time of loop iteration.
            high_resolution_clock::time_point start = high_resolution_clock::now();
            high_resolution_clock::time_point end = high_resolution_clock::now();

            double duration = 0.0; // Duration double for storing execution duration

            struct timespec deadline; // timespec struct for storing time that execution thread should sleep for

            long long iteration_counter = 0; // Iteration counter of the timed loop used for calculating current loop "time" and debugging
            const int update_interval = 1000; // in microseconds


            // I wasted a lot of time on this, so note to self: The code for setting up a UDP Client and a UDP Server are *very* different, or at least produce very different results.
            // Be sure to use the code below for anything that *sends* the messages out and code from the controller side for stuff receiving and replying to a message!
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
			servaddr.sin_port = htons(sim_state_port); 
			servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
			
			int n;
			socklen_t len = sizeof(servaddr);

            while(true) {
                start = high_resolution_clock::now();
                
                std::stringstream msg;
                msg << world->IsPaused() << "|" << world->SimTime().Double();
                sendto(sockfd, (const char *)msg.str().c_str(), strlen(msg.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
                // std::cout << msg.str() << std::endl;
                
                iteration_counter++; // Increment iteration counter
                end = high_resolution_clock::now();
                // This timed loop approach calculates the execution time of the current iteration,
                // then calculates the remaining time for the loop to run at the desired frequency and waits this duration.
                duration = duration_cast<microseconds>(end - start).count();

                long long remainder = (update_interval - duration) * 1e+03;
                // deadline.tv_nsec = remainder;
                // deadline.tv_sec = 0;
                // clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
            }
        }
        
        public: physics::WorldPtr world;

        public: std::thread reset_thread;

        public: void resetWorld() {
            struct timeval tv;
			tv.tv_sec = 1; 
			tv.tv_usec = 0;

			int sockfd, portno, n;
			struct sockaddr_in serv_addr;
			struct hostent *server;

			const int buffer_size = 128;

			char buffer[buffer_size];
			portno = 422000;
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
			bcopy((char *)server->h_addr, 
				(char *)&serv_addr.sin_addr.s_addr,
				server->h_length);
			serv_addr.sin_port = htons(portno);

			if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
				std::cerr << "Error connecting to server.\n";
			
			setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

            while(true) {

                bzero(buffer, buffer_size);
				n = read(sockfd, buffer, buffer_size - 1);
				if (n < 0) {
					std::cout << "Error while reading from socket.\n";
				}
				std::string parsedString(buffer);

				if(parsedString == "1") {
					std::cout << "Reset triggered.\n";
                    world->Reset();

                    ignition::math::Pose3d test(ignition::math::Vector3d(0, 0, 0.01), ignition::math::Quaterniond(0, 0, 0));
                    world->ModelByName("biped")->SetWorldPose(test);
                    world->ModelByName("biped")->SetAngularVel(ignition::math::Vector3d(0, 0, 0));

                    gazebo::physics::JointPtr leftHip3Joint = world->ModelByName("biped")->GetJoint("simplified_biped::left_hip_axis_3_hip_axis_2_joint");
                    gazebo::physics::JointPtr leftHip2Joint = world->ModelByName("biped")->GetJoint("simplified_biped::left_hip_axis_2_hip_axis_1_joint");
                    gazebo::physics::JointPtr leftHip1Joint = world->ModelByName("biped")->GetJoint("simplified_biped::left_hip_axis_1_upper_leg_joint");
                    gazebo::physics::JointPtr leftKneeJoint = world->ModelByName("biped")->GetJoint("simplified_biped::left_knee_lower_leg_joint");
                    gazebo::physics::JointPtr leftAnkleJoint = world->ModelByName("biped")->GetJoint("simplified_biped::left_ankle_foot_base_joint");

                    gazebo::physics::JointPtr rightHip3Joint = world->ModelByName("biped")->GetJoint("simplified_biped::right_hip_axis_3_hip_axis_2_joint");
                    gazebo::physics::JointPtr rightHip2Joint = world->ModelByName("biped")->GetJoint("simplified_biped::right_hip_axis_2_hip_axis_1_joint");
                    gazebo::physics::JointPtr rightHip1Joint = world->ModelByName("biped")->GetJoint("simplified_biped::right_hip_axis_1_upper_leg_joint");
                    gazebo::physics::JointPtr rightKneeJoint = world->ModelByName("biped")->GetJoint("simplified_biped::right_knee_lower_leg_joint");
                    gazebo::physics::JointPtr rightAnkleJoint = world->ModelByName("biped")->GetJoint("simplified_biped::right_ankle_foot_base_joint");
                    
                    world->ModelByName("biped")->GetJointController()->SetPositionPID(leftHip3Joint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftHip2Joint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftHip1Joint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftKneeJoint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftAnkleJoint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));

					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftHip3Joint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftHip2Joint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftHip1Joint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftKneeJoint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftAnkleJoint->GetScopedName(), gazebo::common::PID(1, 0.1, 0.01));

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

                    world->ModelByName("biped")->GetJointController()->SetPositionPID(leftHip3Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftHip2Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftHip1Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftKneeJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetPositionPID(leftAnkleJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));

					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftHip3Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftHip2Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftHip1Joint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftKneeJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));
					world->ModelByName("biped")->GetJointController()->SetVelocityPID(leftAnkleJoint->GetScopedName(), gazebo::common::PID(0, 0, 0));

                    std::this_thread::sleep_for(std::chrono::milliseconds(10));

                    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    world->SetPaused(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
                    world->SetPaused(false);
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // world->SetPaused(true);
                    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    // world->SetPaused(false);
                }
            }
        }

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            world = _parent;
            sim_state_update_thread = std::thread([this] {update_sim_state(); });
            // reset_thread = std::thread([this] {resetWorld(); });
        }
    };

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimStatePlugin)
}

#endif
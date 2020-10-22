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
                deadline.tv_nsec = remainder;
                deadline.tv_sec = 0;
                clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
            }
        }
        
        public: physics::WorldPtr world;

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            world = _parent;
            sim_state_update_thread = std::thread([this] {update_sim_state(); });
        }
    };

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimStatePlugin)
}
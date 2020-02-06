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

using namespace std;
using namespace std::chrono;

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class BipedPlugin : public ModelPlugin
  {

	private: std::thread leftLegStateThread;
	private: std::thread rightLegStateThread;
	private: std::thread leftLegTorqueThread;
	private: std::thread rightLegTorqueThread;

	public: gazebo::physics::ModelPtr model;

	public: gazebo::physics::JointPtr leftHip3Joint;
	public: gazebo::physics::JointPtr leftHip2Joint;
	public: gazebo::physics::JointPtr leftHip1Joint;
	public: gazebo::physics::JointPtr leftKneeJoint;
	public: gazebo::physics::JointPtr leftAnkleJoint;

	public: gazebo::physics::JointPtr rightHip3Joint;
	public: gazebo::physics::JointPtr rightHip2Joint;
	public: gazebo::physics::JointPtr rightHip1Joint;
	public: gazebo::physics::JointPtr rightKneeJoint;
	public: gazebo::physics::JointPtr rightAnkleJoint;

	public: double torqueApplyingInterval = 1000; //microseconds
	public: double statePublishingInterval = 1000; //microseconds
	
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
		this->model = _model;

		std::cout << _model->GetScopedName() << std::endl;

		this->leftHip3Joint = this->model->GetJoint("simplified_biped::left_hip_axis_3_hip_axis_2_joint");
		this->leftHip2Joint = this->model->GetJoint("simplified_biped::left_hip_axis_2_hip_axis_1_joint");
		this->leftHip1Joint = this->model->GetJoint("simplified_biped::left_hip_axis_1_upper_leg_joint");
		this->leftKneeJoint = this->model->GetJoint("simplified_biped::left_knee_lower_leg_joint");
		this->leftAnkleJoint = this->model->GetJoint("simplified_biped::left_ankle_foot_base_joint");

		this->rightHip3Joint = this->model->GetJoint("simplified_biped::right_hip_axis_3_hip_axis_2_joint");
		this->rightHip2Joint = this->model->GetJoint("simplified_biped::right_hip_axis_2_hip_axis_1_joint");
		this->rightHip1Joint = this->model->GetJoint("simplified_biped::right_hip_axis_1_upper_leg_joint");
		this->rightKneeJoint = this->model->GetJoint("simplified_biped::right_knee_lower_leg_joint");
		this->rightAnkleJoint = this->model->GetJoint("simplified_biped::right_ankle_foot_base_joint");

		std::cout << "hip3 joint pos: " << this->leftHip2Joint->Position() << std::endl;

		std::cerr << "\nThe biped plugin is attached to model[" << _model->GetName() << "]\n";

		leftLegStateThread = std::thread(std::bind(&BipedPlugin::PublishLeftLegState, this));	
		rightLegStateThread = std::thread(std::bind(&BipedPlugin::PublishRightLegState, this));
		leftLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyLeftLegTorques, this));
		rightLegTorqueThread = std::thread(std::bind(&BipedPlugin::ApplyRightLegTorques, this));
	}

	public: void PublishLeftLegState() {
		zmq::context_t context(1);
    	zmq::socket_t leftLegPublisher(context, ZMQ_PUB);

		leftLegPublisher.setsockopt(ZMQ_SNDHWM, 100);
    	leftLegPublisher.bind("tcp://*:1337");

		high_resolution_clock::time_point t1 = high_resolution_clock::now();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();

		double duration = 0.0f;

		while(true) {
			t2 = high_resolution_clock::now();
			duration = duration_cast<microseconds>(t2 - t1).count();
			
			if(duration >= statePublishingInterval) {
				t1 = high_resolution_clock::now();

				std::stringstream s;
				s << "lState " << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" << leftAnkleJoint->Position() << "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" << leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);
				
				auto msg = s.str();
				zmq::message_t message(msg.length());
				// Copy Into Buffer
				memcpy(message.data(), msg.c_str(), msg.length());
				// Send It
				leftLegPublisher.send(message);	

				//std::cout << "left:" << s.str() << std::endl;
			}
		}
	}

	public: void PublishRightLegState() {
		zmq::context_t context(1);	
		zmq::socket_t rightLegPublisher(context, ZMQ_PUB);

		rightLegPublisher.setsockopt(ZMQ_SNDHWM, 100);
    	rightLegPublisher.bind("tcp://*:1338");

		high_resolution_clock::time_point t1 = high_resolution_clock::now();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();

		double duration = 0.0f;

		while(true) {
			t2 = high_resolution_clock::now();
			duration = duration_cast<microseconds>(t2 - t1).count();

			if(duration >= statePublishingInterval) {
				t1 = high_resolution_clock::now();

				std::stringstream s;
				s << "rState " << leftHip3Joint->Position() << "|" << leftHip2Joint->Position() << "|" << leftHip1Joint->Position() << "|" << leftKneeJoint->Position() << "|" << leftAnkleJoint->Position() << "|" << leftHip3Joint->GetVelocity(0) << "|" << leftHip2Joint->GetVelocity(0) << "|" << leftHip1Joint->GetVelocity(0) << "|" << leftKneeJoint->GetVelocity(0) << "|" << leftAnkleJoint->GetVelocity(0);

				auto msg = s.str();
				zmq::message_t message(msg.length());
				// Copy Into Buffer
				memcpy(message.data(), msg.c_str(), msg.length());
				// Send It
				rightLegPublisher.send(message);

				//std::cout << "right:" << s.str() << std::endl;
			}
		}
	}

	public: void ApplyLeftLegTorques() {

		zmq::context_t context(1);
    	zmq::socket_t subscriber (context, ZMQ_SUB);
    
    	subscriber.connect("tcp://127.0.0.1:42000");
    	subscriber.setsockopt(ZMQ_SUBSCRIBE, "lTorques", 0);

		high_resolution_clock::time_point t1 = high_resolution_clock::now();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();

		double duration = 0.0f;

		while(true) {
			t2 = high_resolution_clock::now();
            duration = duration_cast<microseconds>( t2 - t1 ).count();

			if(duration >= torqueApplyingInterval) {

				t1 = high_resolution_clock::now();

				zmq::message_t msg;
				subscriber.recv(&msg);

				std::string data = std::string(static_cast<char*>(msg.data()), msg.size());

				std::cout << data << std::endl;

				std::vector<std::string> data_no_topic = split_string(data, ' ');

				std::vector<std::string> torques = split_string(data_no_topic[1], '|');

				if(static_cast<int>(torques.size()) >= 4) {

					double tau_1 = atof(torques[0].c_str());
					double tau_2 = atof(torques[1].c_str());
					double tau_3 = atof(torques[2].c_str());
					double tau_4 = atof(torques[3].c_str());
					double tau_5 = atof(torques[4].c_str());

					this->model->GetJointController()->SetForce(this->leftHip3Joint->GetScopedName(), tau_1);
					this->model->GetJointController()->SetForce(this->leftHip2Joint->GetScopedName(), tau_2);
					this->model->GetJointController()->SetForce(this->leftHip1Joint->GetScopedName(), tau_3);
					this->model->GetJointController()->SetForce(this->leftKneeJoint->GetScopedName(), tau_4);
					this->model->GetJointController()->SetForce(this->leftAnkleJoint->GetScopedName(), tau_5);
				}
			}
		}
	}

	public: void ApplyRightLegTorques() {

		zmq::context_t context(1);
    	zmq::socket_t subscriber (context, ZMQ_SUB);
    
    	subscriber.connect("tcp://127.0.0.1:42001");
    	subscriber.setsockopt(ZMQ_SUBSCRIBE, "rTorques", 0);

		high_resolution_clock::time_point t1 = high_resolution_clock::now();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();

		double duration = 0.0f;

		while(true) {
			t2 = high_resolution_clock::now();
            duration = duration_cast<microseconds>( t2 - t1 ).count();

			if(duration >= torqueApplyingInterval) {

				t1 = high_resolution_clock::now();

				zmq::message_t msg;
				subscriber.recv(&msg);

				std::string data = std::string(static_cast<char*>(msg.data()), msg.size());

				std::cout << data << std::endl;

				std::vector<std::string> data_no_topic = split_string(data, ' ');

				std::vector<std::string> torques = split_string(data_no_topic[1], '|');

				if(static_cast<int>(torques.size()) >= 4) {

					double tau_1 = atof(torques[0].c_str());
					double tau_2 = atof(torques[1].c_str());
					double tau_3 = atof(torques[2].c_str());
					double tau_4 = atof(torques[3].c_str());
					double tau_5 = atof(torques[4].c_str());

					this->model->GetJointController()->SetForce(this->rightHip3Joint->GetScopedName(), tau_1);
					this->model->GetJointController()->SetForce(this->rightHip2Joint->GetScopedName(), tau_2);
					this->model->GetJointController()->SetForce(this->rightHip1Joint->GetScopedName(), tau_3);
					this->model->GetJointController()->SetForce(this->rightKneeJoint->GetScopedName(), tau_4);
					this->model->GetJointController()->SetForce(this->rightAnkleJoint->GetScopedName(), tau_5);
				}
			}
		}
	}
	};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(BipedPlugin)
}
#endif

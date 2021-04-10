#ifndef _GAZEBO_FOOT_FORCE_PLUGIN_HH_
#define _GAZEBO_FOOT_FORCE_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <chrono>
using namespace std::chrono;

namespace gazebo
{
    /// \brief An example plugin for a contact sensor.
    class FootForcePlugin : public SensorPlugin
    {
        /// \brief Constructor.
    public:
        FootForcePlugin();

        /// \brief Destructor.
    public:
        virtual ~FootForcePlugin();

    private:
        void update_force();

    private:
        std::thread update_thread;

    private:
        std::mutex force_mutex;

    private:
        const int udp_buffer_size = 4096;

    private:
        int port;

    private:
        ignition::math::Vector3d force;

        /// \brief Load the sensor plugin.
        /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
    public:
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        /// \brief Callback that receives the contact sensor's update signal.
    private:
        virtual void OnUpdate();

        /// \brief Pointer to the contact sensor
    private:
        sensors::ForceTorqueSensorPtr parentSensor;

        /// \brief Connection that maintains a link between the contact sensor's
        /// updated signal and the OnUpdate callback.
    private:
        event::ConnectionPtr updateConnection;
    };
} // namespace gazebo
#endif
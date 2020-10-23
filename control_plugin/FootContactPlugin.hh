#ifndef _GAZEBO_FOOT_CONTACT_PLUGIN_HH_
#define _GAZEBO_FOOT_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <chrono>
using namespace std::chrono;

namespace gazebo
{
    /// \brief An example plugin for a contact sensor.
    class FootContactPlugin : public SensorPlugin
    {
        /// \brief Constructor.
    public:
        FootContactPlugin();

        /// \brief Destructor.
    public:
        virtual ~FootContactPlugin();

    private:
        void update_state();

    private:
        std::thread update_thread;

    private:
        std::mutex state_mutex;

    private:
        static const int udp_buffer_size = 4096;

    private:
        static int port;

    private:
        bool state;

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
        sensors::ContactSensorPtr parentSensor;

        /// \brief Connection that maintains a link between the contact sensor's
        /// updated signal and the OnUpdate callback.
    private:
        event::ConnectionPtr updateConnection;
    };
} // namespace gazebo
#endif
#ifndef msr_airlib_BetaflightDroneController_hpp
#define msr_airlib_BetaflightDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"

#include "UdpSocket.hpp"

namespace msr
{
namespace airlib
{

    class BetaflightApi : public MultirotorApiBase
    {

    public:
        BetaflightApi(const MultiRotorParams* vehicle_params, const AirSimSettings::MavLinkConnectionInfo& connection_info)
            : connection_info_(connection_info), vehicle_params_(vehicle_params)
        {
            sensors_ = &getSensors();
            start_time_ = ClockFactory::get()->nowNanos();
            resetRotors();
            connect();
        }

        ~BetaflightApi()
        {
            closeConnections();
        }

    public:
        virtual void resetImplementation() override
        {
            MultirotorApiBase::resetImplementation();

            // Reset state
        }

        // Update sensor data & send to betaflight
        virtual void update() override
        {
            MultirotorApiBase::update();

            // Time
            double timestamp = static_cast<double>(ClockFactory::get()->nowNanos() - start_time_) / 1E9;

            if (timestamp >= start_timestamp) {

                if (failed_pkts == 0) sendState(timestamp);
                
                if (recvControl()) {

                    failed_pkts = 0;
                    if (!is_connected_) is_connected_ = true;
                }
                else {

                    failed_pkts++;

                    if (failed_pkts >= max_failed_pkts) {
                    
                        failed_pkts = 0;
                        is_connected_ = false;
                        start_timestamp = timestamp + wait_time;
                        resetRotors();
                        Utils::log(Utils::stringf("Rotors data not recieved. Retry in %.0f sec", wait_time), Utils::kLogLevelWarn);
                    }
                }
            }
        }

        // necessary overrides
        virtual bool isApiControlEnabled() const override
        {
            return true;
        }
        virtual void enableApiControl(bool is_enabled) override
        {
            // Utils::log("Not Implemented: enableApiControl", Utils::kLogLevelInfo);
            unused(is_enabled);
        }
        virtual bool armDisarm(bool arm) override
        {
            unused(arm);
            return is_connected_;
        }
        virtual GeoPoint getHomeGeoPoint() const override
        {
            return physics_->getEnvironment().getHomeGeoPoint();
        }
        virtual void getStatusMessages(std::vector<std::string>& messages) override
        {
            unused(messages);
        }

        virtual const SensorCollection& getSensors() const override
        {
            return vehicle_params_->getSensors();
        }

    public: //TODO:MultirotorApiBase implementation
        virtual real_T getActuation(unsigned int rotor_index) const override
        {
            return rotor_controls_[rotor_index];
        }

        virtual size_t getActuatorCount() const override
        {
            return vehicle_params_->getParams().rotor_count;
        }

        virtual void moveByRC(const RCData& rc_data) override
        {
            unused(rc_data);
            // setRCData(rc_data);
        }

        virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
        {
            //Utils::log("Not Implemented: setSimulatedGroundTruth", Utils::kLogLevelInfo);
            unused(kinematics);
            unused(environment);

            if (physics_ == nullptr) {
                physics_ = static_cast<MultiRotorPhysicsBody*>(this->getParent());
            }
        }

        virtual bool setRCData(const RCData& rc_data) override
        {
            last_rcData_ = rc_data;
            is_rc_connected_ = true;

            return true;
        }

    protected:
        virtual Kinematics::State getKinematicsEstimated() const override
        {
            return physics_->getKinematics();
        }

        virtual LandedState getLandedState() const override
        {
            return physics_->isGrounded() ? LandedState::Landed : LandedState::Flying;
        }

        virtual RCData getRCData() const override
        {
            //return what we received last time through setRCData
            return last_rcData_;
        }

        virtual GeoPoint getGpsLocation() const override
        {
            return physics_->getEnvironment().getState().geo_point;
        }

        virtual float getCommandPeriod() const override
        {
            return 1.0f / 50; //50hz
        }

        virtual float getTakeoffZ() const override
        {
            // pick a number, 3 meters is probably safe
            // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
            // return params_.takeoff.takeoff_z;
            return 3.0;
        }

        virtual float getDistanceAccuracy() const override
        {
            return 0.5f; //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
        }

        virtual void setControllerGains(uint8_t controllerType, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
        {
            unused(controllerType);
            unused(kp);
            unused(ki);
            unused(kd);
            Utils::log("Not Implemented: setControllerGains", Utils::kLogLevelInfo);
        }

        virtual void commandMotorPWMs(float front_right_pwm, float front_left_pwm, float rear_right_pwm, float rear_left_pwm) override
        {
            unused(front_right_pwm);
            unused(front_left_pwm);
            unused(rear_right_pwm);
            unused(rear_left_pwm);
            Utils::log("Not Implemented: commandMotorPWMs", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw_rate);
            unused(throttle);
            Utils::log("Not Implemented: commandRollPitchYawrateThrottle", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw);
            unused(z);
            Utils::log("Not Implemented: commandRollPitchYawZ", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw);
            unused(throttle);
            Utils::log("Not Implemented: commandRollPitchYawThrottle", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw_rate);
            unused(z);
            Utils::log("Not Implemented: commandRollPitchYawrateZ", Utils::kLogLevelInfo);
        }

        virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) override
        {
            unused(roll_rate);
            unused(pitch_rate);
            unused(yaw_rate);
            unused(z);
            Utils::log("Not Implemented: commandAngleRatesZ", Utils::kLogLevelInfo);
        }

        virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) override
        {
            unused(roll_rate);
            unused(pitch_rate);
            unused(yaw_rate);
            unused(throttle);
            Utils::log("Not Implemented: commandAngleRatesZ", Utils::kLogLevelInfo);
        }

        virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
        {
            unused(vx);
            unused(vy);
            unused(vz);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandVelocity", Utils::kLogLevelInfo);
        }

        virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
        {
            unused(vx);
            unused(vy);
            unused(z);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandVelocityZ", Utils::kLogLevelInfo);
        }

        virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
        {
            unused(x);
            unused(y);
            unused(z);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandPosition", Utils::kLogLevelInfo);
        }

        virtual const MultirotorApiParams& getMultirotorApiParams() const override
        {
            return safety_params_;
        }

    protected:
        void closeConnections()
        {
            if (motor_socket_ != nullptr)
                motor_socket_->close();

            if (servo_socket_ != nullptr)
                servo_socket_->close();
        }

        void connect()
        {
            ip_ = connection_info_.udp_address;

            closeConnections();

            if (ip_ == "") {
                throw std::invalid_argument("UdpIp setting is invalid.");
            }

            Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for receiving servo position", servo_port_, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);
            Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for receiving rotor power", motor_port_, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);
            Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", state_port_, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);

            motor_socket_ = std::make_unique<mavlinkcom::UdpSocket>();
            motor_socket_->bind(connection_info_.local_host_ip, motor_port_);

            // servo_socket_ = std::make_unique<mavlinkcom::UdpSocket>();
            // servo_socket_->bind(connection_info_.local_host_ip, servo_port_);
        }

        void resetRotors()
        {
            for (auto i = 0; i < kBetaflightMotorCount; i++)
                rotor_controls_[i] = 0;
        }

    private:
        // actual functions

        void sendState(double timestamp) // send fdmPacket to betaflight i.e udp:9002
        {
            if (sensors_ == nullptr || motor_socket_ == nullptr)
                return;
            
            FdmPacket pkt = FdmPacket();

            // IMU
            const auto& imu_output = getImuData("");

            // Baro
            const auto& baro_output = getBarometerData("");

            // Time
            pkt.timestamp = timestamp;

            // Angular Velocity
            pkt.imu_angular_velocity_rpy[0] = imu_output.angular_velocity[0];
            pkt.imu_angular_velocity_rpy[1] = imu_output.angular_velocity[1];
            pkt.imu_angular_velocity_rpy[2] = imu_output.angular_velocity[2];

            // Linear Acceleration
            pkt.imu_linear_acceleration_xyz[0] = imu_output.linear_acceleration[0];
            pkt.imu_linear_acceleration_xyz[1] = imu_output.linear_acceleration[1];
            pkt.imu_linear_acceleration_xyz[2] = imu_output.linear_acceleration[2];

            // Orientation Quaternion. In case USE_IMU_CALC is not defined on betaflight side
            pkt.imu_orientation_quat[0] = imu_output.orientation.w();
            pkt.imu_orientation_quat[1] = imu_output.orientation.x();
            pkt.imu_orientation_quat[2] = imu_output.orientation.y();
            pkt.imu_orientation_quat[3] = imu_output.orientation.z();

            // Position
            pkt.position_xyz[0] = 0;
            pkt.position_xyz[1] = 0;
            pkt.position_xyz[2] = 0;

            // Velocity
            pkt.velocity_xyz[0] = 0;
            pkt.velocity_xyz[1] = 0;
            pkt.velocity_xyz[2] = 0;

            // Pressure
            pkt.pressure = baro_output.pressure;

            motor_socket_->sendto(&pkt, sizeof(pkt), ip_, state_port_);
        }

        bool recvControl()
        {
            ServoPacket s_pkt;
            int s_recv_ret = motor_socket_->recv(&s_pkt, sizeof(s_pkt), 100);

            if (s_recv_ret != sizeof(s_pkt)) {
                if (s_recv_ret <= 0) {
                    Utils::log(Utils::stringf("Error while receiving rotor control data - ErrorNo: %d", s_recv_ret), Utils::kLogLevelInfo);
                }
                else {
                    Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes for rotors", s_recv_ret, sizeof(s_pkt)), Utils::kLogLevelInfo);
                }

                return false;
            }

            for (auto i = 0; i < kBetaflightMotorCount; ++i) {
                rotor_controls_[i] = s_pkt.pwm[i];
            }

            // ServoPacketRaw sr_pkt;
            // int sr_recv_ret = servo_socket_->recv(&sr_pkt, sizeof(sr_pkt), 100);

            // if (sr_recv_ret != sizeof(sr_pkt)) {
            //     if (sr_recv_ret <= 0) {
            //         Utils::log(Utils::stringf("Error while receiving servo control data - ErrorNo: %d", sr_recv_ret), Utils::kLogLevelInfo);
            //     }
            //     else {
            //         Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes for servos", sr_recv_ret, sizeof(sr_pkt)), Utils::kLogLevelInfo);
            //     }
            // }
            // else {

            //     for (auto i = 0; i < kBetaflightServoCount; ++i) {
            //         servo_controls_[i] = sr_pkt.pwm[i];
            //     }
            // }

            // Utils::log(Utils::stringf("pwm received: [ %f, %f, %f, %f ]", rotor_controls_[0], rotor_controls_[1], rotor_controls_[2], rotor_controls_[3]), Utils::kLogLevelInfo);
            return true;
        }

    private:
        struct FdmPacket // equivalent of fdm_packet in betaflight SITL
        {
            double timestamp;                       // in seconds
            double imu_angular_velocity_rpy[3];     // rad/s -> range: +/- 8192; +/- 2000 deg/se
            double imu_linear_acceleration_xyz[3];  // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
            double imu_orientation_quat[4];         //w, x, y, z
            double velocity_xyz[3];                 // m/s, earth frame
            double position_xyz[3];                 // meters, NED from origin
            double pressure;                        // pascals, atmosphere pressure
        };

        static const int kBetaflightMotorCount = 4;
        static const int kBetaflightServoCount = 8;

        static const int kBetaflightMaxRCCount = 16;

        struct ServoPacket // equivalent of servo_packet in betaflight SITL
        {
            float pwm[kBetaflightMotorCount];
        };

        struct ServoPacketRaw // equivalent of servo_packet_raw in betaflight SITL
        {
            uint16_t motorCount;                // Count of motor in the PWM output.
            float pwm[kBetaflightServoCount]; // Raw PWM from 1100 to 1900
        };

        float rotor_controls_[kBetaflightMotorCount];
        float servo_controls_[kBetaflightServoCount];

        // Sockets
        std::unique_ptr<mavlinkcom::UdpSocket> motor_socket_;
        std::unique_ptr<mavlinkcom::UdpSocket> servo_socket_;

        const uint16_t servo_port_ = 9001;
        const uint16_t motor_port_ = 9002;
        const uint16_t state_port_ = 9003;
        const uint16_t rc_port_ = 9004;

        MultirotorApiParams safety_params_;
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        const MultiRotorParams* vehicle_params_;
        const MultiRotorPhysicsBody* physics_ = nullptr;
        const SensorCollection* sensors_;

        std::string ip_;
        RCData last_rcData_;
        bool is_connected_ = false;
        bool is_rc_connected_;
        uint64_t start_time_; // changed in contructor call

        uint8_t failed_pkts = 0;
        const uint8_t max_failed_pkts = 5;  // maximum amount of failed packets

        double start_timestamp = 0;
        double wait_time = 5; 
    };
}
} // namespace

#endif
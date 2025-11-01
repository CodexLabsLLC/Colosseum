#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"

namespace msr
{
    namespace airlib
    {
        MavLinkMultirotorApi::~MavLinkMultirotorApi()
        {
            closeAllConnection();
            if (this->connect_thread_.joinable()) {
                this->connect_thread_.join();
            }
            if (this->telemetry_thread_.joinable()) {
                this->telemetry_thread_.join();
            }
        }
    } // namespace airlib
} // namespace msr

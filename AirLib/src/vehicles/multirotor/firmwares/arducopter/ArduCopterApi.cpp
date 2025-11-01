#include "vehicles/multirotor/firmwares/arducopter/ArduCopterApi.hpp"

namespace msr
{
    namespace airlib
    {
        ArduCopterApi::~ArduCopterApi()
        {
            closeConnections();
        }
    } // namespace airlib
} // namespace msr

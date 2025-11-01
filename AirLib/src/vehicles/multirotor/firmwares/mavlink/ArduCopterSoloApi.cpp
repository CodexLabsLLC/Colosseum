#include "vehicles/multirotor/firmwares/mavlink/ArduCopterSoloApi.hpp"

namespace msr
{
    namespace airlib
    {
        ArduCopterSoloApi::~ArduCopterSoloApi()
        {
            closeAllConnection();
        }
    } // namespace airlib
} // namespace msr

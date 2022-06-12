#pragma once
#include <base/data_type/common_struct.h>

namespace base::ldm {

bool loadKittiCloud(const std::string& file_name, base::LidarMeasurement* lidar_mes);
    
} // namespace io

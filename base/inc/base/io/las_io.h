#pragma once
#include <liblas/liblas.hpp>
//
#include <base/data_type/common_struct.h>

namespace base::ldm {

bool savePointCloudToLas(const base::LidarMeasurement& lidar_mes, const std::string& file_name);
    
} // namespace io

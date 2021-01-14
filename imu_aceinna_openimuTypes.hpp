#ifndef imu_aceinna_openimu_TYPES_HPP
#define imu_aceinna_openimu_TYPES_HPP

#include <imu_aceinna_openimu/Configuration.hpp>
#include <imu_aceinna_openimu/Status.hpp>
#include <imu_aceinna_openimu/FilterState.hpp>

namespace imu_aceinna_openimu {
    /** Main configuration structure
     */
    struct TaskConfiguration {
        std::string period_message = "e2";

        int16_t acceleration_low_pass_filter = 25;
        int16_t angular_velocity_low_pass_filter = 25;

        GPSProtocol gps_protocol = GPS_UBLOX;
        int32_t gps_baudrate = 115200;
    };


    /** Internal IMU status
     */
    struct TaskStatus {
        base::Time time;

        Status imu_status;
        FilterState filter_state;
    };
}

#endif


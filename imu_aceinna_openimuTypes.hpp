#ifndef imu_aceinna_openimu_TYPES_HPP
#define imu_aceinna_openimu_TYPES_HPP

#include <imu_aceinna_openimu/Configuration.hpp>

namespace imu_aceinna_openimu {
    /** Main configuration structure
     */
    struct TaskConfiguration {
        int16_t acceleration_low_pass_filter = 25;
        int16_t angular_velocity_low_pass_filter = 25;

        GPSProtocol gps_protocol = GPS_UBLOX;
        int32_t gps_baud_rate = 115200;

        bool use_gps = true;
        bool use_magnetometers = true;
        bool use_gps_course_as_heading = true;
    };

    /** Configuration of the output periods
     */
    struct Periods {
        /**
         * The master frequency. Maximum frequency is 20 Hz.
         */
        int main = 20;

        /**
         * Pose and acceleration period, in multiple of the main period.
         * Set to zero to disable.
         */
        int pose_and_acceleration = 1;

        /**
         * Raw sensors period, in multiples of the main period.
         * Set to zero to disable
         */
        int raw_sensors = 0;

        /**
         * Status period, in multiples of the main period.
         * Set to zero to disable
         */
        int status = 100;
    };
}

#endif


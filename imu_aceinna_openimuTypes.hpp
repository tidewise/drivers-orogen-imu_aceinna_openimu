#ifndef imu_aceinna_openimu_TYPES_HPP
#define imu_aceinna_openimu_TYPES_HPP

#include <imu_aceinna_openimu/Configuration.hpp>
#include <imu_aceinna_openimu/Status.hpp>
#include <imu_aceinna_openimu/FilterState.hpp>

namespace imu_aceinna_openimu {
    /** Main configuration structure
     */
    struct TaskConfiguration {
        int16_t acceleration_low_pass_filter = 25;
        int16_t angular_velocity_low_pass_filter = 25;

        GPSProtocol gps_protocol = GPS_UBLOX;
        int32_t gps_baudrate = 115200;

        bool use_gps = true;
        bool use_magnetometers = true;
        bool use_gps_course_as_heading = true;
    };


    /** Internal IMU status
     */
    struct TaskStatus {
        base::Time time;

        Status imu_status;
        FilterState filter_state;
    };

    /** Configuration of the output periods
     */
    struct Periods {
        /**
         * Main message rate. All the other messages are expressed as multiples
         * of this main period
         *
         * Allowed values are 200, 100, 50, 25, 20, 10, 5, 2 and 1.
         */
        int main_rate = 50;

        /**
         * Status period, in multiples of the main period. Set
         * to zero to disable
         */
        int pose_and_acceleration = 2;

        /**
         * Raw sensors period, in multiples of the main period
         * Set to zero to disable
         */
        int raw_sensors = 0;

        /**
         * Status period, in multiples of the main period. Set
         * to zero to disable
         */
        int status = 50;
    };
}

#endif


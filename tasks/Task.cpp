/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <imu_aceinna_openimu/Driver.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>
#include <aggregator/TimestampEstimator.hpp>

using namespace imu_aceinna_openimu;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    mUTMConverter.setParameters(_utm_parameters.get());

    iodrivers_base::ConfigureGuard guard(this);
    Driver* driver = new Driver();
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver);

    // This is MANDATORY and MUST be called after the setDriver but before you do
    // anything with the driver
    if (! TaskBase::configureHook())
        return false;

    driver->validateDevice();

    TaskConfiguration conf = _configuration.get();
    driver->writeAccelerationLowPassFilter(
        conf.acceleration_low_pass_filter
    );
    driver->writeAngularVelocityLowPassFilter(
        conf.angular_velocity_low_pass_filter
    );
    driver->writeGPSProtocol(conf.gps_protocol);
    driver->writeGPSBaudrate(conf.gps_baudrate);

    delete mTimestampEstimator;
    mTimestampEstimator = new aggregator::TimestampEstimator(
        base::Time::fromSeconds(10),
        base::Time::fromSeconds(1.0 / _message_rate.get()));

    guard.commit();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    mIMUHasGPSTime = false;
    mTimestampEstimator->reset();
    mLastTimestampEstimatorStatus = base::Time::now();
    Driver* driver = static_cast<Driver*>(mDriver);
    driver->writePeriodicPacketConfiguration(
        _configuration.get().period_message,
        _message_rate.get()
    );
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}

static base::Time ONE_WEEK = base::Time::fromMilliseconds(604800000);

base::Time Task::timeSync(bool imuHasGPSTime, base::Time local_time, base::Time sample_time) {
    bool gpsTimeSwitch = imuHasGPSTime ^ mIMUHasGPSTime;
    mIMUHasGPSTime = imuHasGPSTime;

    if (imuHasGPSTime && _local_time_synchronized_with_gps.get()) {
        // We assume the sample has taken less than one second to reach us,
        // and reset only the sub-second part, taking care of wraparounds
        uint64_t local_time_ms = local_time.toMilliseconds();
        uint64_t local_time_s = local_time_ms / 1000;
        uint64_t sample_time_fraction_ms = sample_time.toMilliseconds() % 1000;

        uint64_t corrected_time_ms = local_time_s * 1000 + sample_time_fraction_ms;
        if (corrected_time_ms > local_time_ms) {
            corrected_time_ms -= 1000;
        }
        base::Time corrected_time(base::Time::fromMilliseconds(corrected_time_ms));
        mTimestampEstimatorStatus.stamp = local_time;
        mTimestampEstimatorStatus.latency = local_time - corrected_time;
        return corrected_time;
    }
    else {
        // Reset the timestamp estimator if we switch between
        if (gpsTimeSwitch) {
            mTimestampEstimator->reset();
        }
        mTimestampEstimatorStatus = mTimestampEstimator->getStatus();
        return mTimestampEstimator->update(local_time);
    }
}
void Task::processIO()
{
    Driver* driver = static_cast<Driver*>(mDriver);

    if (driver->processOne()) {
        auto local_time = base::Time::now();

        auto update = driver->getLastPeriodicUpdate();
        update.computeNWUPosition(mUTMConverter);
        auto time = timeSync(update.filter_state.mode == OPMODE_INS,
                             local_time, update.rbs.time);
        update.rbs.time = time;

        _pose_samples.write(update.rbs);

        if (!update.rba.time.isNull()) {
            update.rba.time = time;
            _acceleration_samples.write(update.rba);
        }

        if (!update.magnetic_info.time.isNull()) {
            update.magnetic_info.time = time;
            _magnetic_info.write(update.magnetic_info);
        }
    }

    bool need_timestamp_estimator_status =
        (base::Time::now() - mLastTimestampEstimatorStatus) >
        _timestamp_estimator_status_period.get();
    if (need_timestamp_estimator_status) {
        _timestamp_estimator_status.write(mTimestampEstimatorStatus);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    Driver* driver = static_cast<Driver*>(mDriver);
    driver->writePeriodicPacketConfiguration("e2", 0);
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

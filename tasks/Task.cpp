/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <imu_aceinna_openimu/Driver.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>

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

    Periods periods = _periods.get();
    driver->writePeriodicPacketConfiguration(
        "e3", periods.pose_and_acceleration);

    guard.commit();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::processIO()
{
    Driver* driver = static_cast<Driver*>(mDriver);
    if (driver->processOne() == Driver::UPDATED_STATE) {
        auto state = driver->getState();
        state.computeNWUPosition(mUTMConverter);
        _pose_samples.write(state.rbs);
        _acceleration_samples.write(state.rba);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

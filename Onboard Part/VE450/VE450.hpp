#ifndef VE450_HPP
#define VE450_HPP

// System Includes
#include <cmath>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

//!@note: All the default timeout parameters are for acknowledgement packets
//! from the aircraft.

/*! Monitored Takeoff
    This implementation of takeoff with monitoring makes sure your aircraft
    actually took off and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/

bool monitoredTakeoff(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

// Examples of commonly used Flight Mode APIs

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/

bool moveByPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired,
                          float yOffsetDesired, float zOffsetDesired,
                          float yawDesired, float posThresholdInM = 0.5,
                          float yawThresholdInDeg = 1.0);

/*! Monitored Landing (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.

!*/

bool monitoredLanding(DJI::OSDK::Vehicle *vehiclePtr, int timeout = 1);

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 * coordinates.
 *
 * Accurate when distances are small.
!*/

void localOffsetFromGpsOffset(DJI::OSDK::Vehicle*             vehicle,
                              DJI::OSDK::Telemetry::Vector3f& deltaNed,
                              void* target, void* origin);

DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* quaternionData);

bool startGlobalPositionBroadcast(DJI::OSDK::Vehicle* vehicle);

// CAUTION: Super unsafe!!!!!! Only for testing with blades removed
bool TakeoffAnyway(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);
bool LandingAnyway(DJI::OSDK::Vehicle *vehiclePtr, int timeout = 1);

#endif // VE450_HPP

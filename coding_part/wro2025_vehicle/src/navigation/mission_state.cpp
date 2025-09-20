// mission_state.cpp
// Implementation of MissionState

#include "mission_state.h"
#include <iostream> // For potential logging/debugging

MissionState::MissionState()
    : challengeType(ChallengeType::UNKNOWN),
      phase(MissionPhase::PRE_START),
      startPosition(StartPosition::UNKNOWN),
      currentLap(0),
      lapCompleted(false),
      inStartSection(false),
      nearParkingLot(false)
{
    // Constructor initializes all state variables to known default values
}

void MissionState::setPhase(MissionPhase newPhase)
{
    // Optional: Add logging here to track state transitions
    // std::cout << "Mission State Transition: " << static_cast<int>(phase) << " -> " << static_cast<int>(newPhase) << std::endl;
    phase = newPhase;
}
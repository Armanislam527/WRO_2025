// mission_state.h
// Tracks the overall state of the mission (Open/Obstacle, phase, scoring conditions)

#ifndef MISSION_STATE_H
#define MISSION_STATE_H

#include <cstdint>

enum class ChallengeType
{
    UNKNOWN,
    OPEN_CHALLENGE,
    OBSTACLE_CHALLENGE
};

enum class MissionPhase
{
    PRE_START,         // Before physical button press
    WAITING_FOR_GO,    // After button press, waiting for Pi's "GO" command
    DRIVING_LAPS,      // Driving the 3 main laps
    FINDING_PARKING,   // (Obstacle only) Finding the parking lot entrance
    PARKING,           // (Obstacle only) Executing the parking maneuver
    STOPPING_AT_START, // Stopping in the start section (Open) or finishing (Obstacle)
    COMPLETED,         // Mission successfully finished
    EMERGENCY_STOPPED  // Stopped due to E-Stop or error
};

enum class StartPosition
{
    UNKNOWN,
    REGULAR_ZONE, // Started in the middle zone above parking lot
    PARKING_LOT   // Started inside the parking lot
};

class MissionState
{
public:
    MissionState();

    // Getters
    ChallengeType getChallengeType() const { return challengeType; }
    MissionPhase getPhase() const { return phase; }
    StartPosition getStartPosition() const { return startPosition; }
    int getCurrentLap() const { return currentLap; }
    bool isLapCompleted() const { return lapCompleted; }
    bool isInStartSection() const { return inStartSection; }
    bool isNearParkingLot() const { return nearParkingLot; } // For Obstacle challenge

    // Setters (Called by other modules)
    void setChallengeType(ChallengeType type) { challengeType = type; }
    void setPhase(MissionPhase newPhase);
    void setStartPosition(StartPosition pos) { startPosition = pos; }
    void setCurrentLap(int lap) { currentLap = lap; }
    void setLapCompleted(bool completed) { lapCompleted = completed; }
    void setInStartSection(bool inStart) { inStartSection = inStart; }
    void setNearParkingLot(bool near) { nearParkingLot = near; }

    // Utility functions
    bool isObstacleChallenge() const { return challengeType == ChallengeType::OBSTACLE_CHALLENGE; }
    bool isOpenChallenge() const { return challengeType == ChallengeType::OPEN_CHALLENGE; }
    bool isDrivingPhase() const
    {
        return phase == MissionPhase::DRIVING_LAPS ||
               phase == MissionPhase::FINDING_PARKING ||
               phase == MissionPhase::PARKING ||
               phase == MissionPhase::STOPPING_AT_START;
    }

private:
    ChallengeType challengeType;
    MissionPhase phase;
    StartPosition startPosition;

    int currentLap;      // 0-based (0, 1, 2)
    bool lapCompleted;   // Flag for lap completion logic
    bool inStartSection; // Flag set by lap counter/navigation when in start zone
    bool nearParkingLot; // Flag set by navigation when approaching parking lot (Obstacle)
};

#endif // MISSION_STATE_H
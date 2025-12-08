stateDiagram-v2
START --> IDLE

    START: declare state,                                                           declare driveResult(NONE, ARRIVED_OK, ALIGN_FAILED, PATH_FAILED, ABORTED)                                                                          declare driveGoal(NONE, GOTO_BALL_SPOT, GOTO_SHOOT_SPOT, GOTO_PARK, PURE_ALIGN_TAG)

    state IDLE {
        IDLE_WAIT: wait for startCycle()
    }

    IDLE --> MOVING: startCycle(goalKind,targetPose,timeout)

    state MOVING {
     %%   [*] --> MOVING_ACTIVE
        MOVING_ACTIVE: drive toward target using odo + IMU
    }

    %% From MOVING we either reach target or fail
    MOVING --> ALIGNING: reached target position        goalKinds is right                  state = ALIGNING.       result = ARRIVED_OK

    MOVING --> DONE: timeout OR large position error result = PATH_FAILED

    %% ALIGNING step is optional, depending on goalKind
    state ALIGNING {
%%        [*] --> ALIGNING_ACTIVE
ALIGNING_ACTIVE: camera-based fine alignment (e.g. AprilTag or local pose)
}

    ALIGNING --> DONE: alignment successful      result = ALIGNED

    ALIGNING --> DONE: alignment timeout or unstable   result = ALIGN_FAILED

    state DONE {
        [*] --> DONE_WAIT
        DONE_WAIT: expose DriveResult to GameManager
    }

    DONE --> IDLE: resetCycle()  state = IDLE, result = NONE

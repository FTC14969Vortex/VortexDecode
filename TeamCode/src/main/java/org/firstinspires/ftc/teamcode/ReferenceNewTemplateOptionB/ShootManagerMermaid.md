stateDiagram-v2
[*] --> IDLE

    state IDLE {
        [*] --> IDLE_WAIT
        IDLE_WAIT: flywheel stopped kicker/flipper in intake-safe pose  wait for startCycle()
    }

    IDLE --> SPINNING: startCycle(targetShots, distanceInches, timeoutSec) result = NONE, shotsFired = 0

    state SPINNING {
        [*] --> SPINNING_UP
        SPINNING_UP: compute target velocity from distance    ramp up flywheel (waitForFlyWheelShootingVelocity)
    }

    %% Flywheel at speed, alignment is already OK (or handled by drive)
    SPINNING --> SHOOTING: velocity within tolerance  (and alignment OK from higher-level)   open gate for first shot

    %% Jam or spin-up failure
    SPINNING --> DONE: timeout OR jam detected    result = JAM_FAILED
        

    state SHOOTING {
        [*] --> BURST_START
        BURST_START: sequence flipper angles  and gate timing for multiple balls
    }

    SHOOTING --> SHOOTING: more balls pending && spin still OK   no jam detected     shotsFired++

    SHOOTING --> DONE:  all requested shots fired   result = SUCCESS

    SHOOTING --> DONE: lost speed too much OR jam mid-burst   result = JAM_FAILED

    SHOOTING --> DONE:  abortCycle() (e.g. time almost over)    result = ABORTED

    state DONE {
        [*] --> DONE_WAIT
        DONE_WAIT: stop flywheel  reset flipper + kicker  expose ShooterResult to GameManager
    }

    DONE --> IDLE: resetCycle()   state = IDLE  result = NONE


stateDiagram-v2
START --> INIT

    START : declare driveState, driveResult(NONE, ARRIVED_OK, ALIGNED, ALIGNED_FAILED, PATH_FAILED, ABORTED) 

    %% ------------------------
    %% INIT
    %% ------------------------
    state INIT {
        INIT_VARS: init BALL_SPOTS[], SHOOT_SPOTS[],\nPARK_POS, ballIndex=0, shootIndex=0
        INIT_VARS --> INIT_DONE
    }

    INIT --> PLAN_NEXT

    %% ------------------------
    %% PLAN_NEXT  (high-level decision)
    %% ------------------------
    state PLAN_NEXT {
        DECIDE: check remaining time, ballIndex, shootIndex, robot state
    }

    PLAN_NEXT --> DRIVE_TO_BALL :  timeEnough()  &&    hasMoreBallSpots()
    PLAN_NEXT --> DRIVE_TO_SHOOT : hasLoadedBalls() && timeEnoughForShoot()

    PLAN_NEXT --> PARK : !timeEnough() || allWorkDone()

    %% ------------------------
    %% DRIVE_TO_BALL  (uses DriveController)
    %% ------------------------
    state DRIVE_TO_BALL {
        DRIVE_BALL_CMD: drive.resetCycle()   drive.startCycle(GOTO_BALL_SPOT    BALL_SPOTS[ballIndex], timeout)
        DRIVE_BALL_CMD --> DRIVE_BALL_WAIT
        DRIVE_BALL_WAIT: wait until drive.state == DONE
    }

    DRIVE_TO_BALL --> INTAKE: drive.result == ARRIVED_OK

    DRIVE_TO_BALL --> PLAN_NEXT:  drive.result == PATH_FAILED ||\n drive.result == ABORTED\n/ ballIndex++

    %% ------------------------
    %% INTAKE  (uses IntakeController)
    %% ------------------------
    state INTAKE {
        INTAKE_CMD: intake.resetCycle() \nintake.startCycle(targetBallsForSpot,\nintakeTimeout)
        INTAKE_CMD --> INTAKE_WAIT
        INTAKE_WAIT: wait until intake.state == DONE
    }

    %% got ≥1 ball
    INTAKE --> DRIVE_TO_SHOOT: intake.result == GOT_BALLS

    %% no ball, still enough time → skip this spot
    INTAKE --> PLAN_NEXT: intake.result == NO_BALL && timeEnough()\n/ ballIndex++

    %% no ball or aborted, not enough time → park
    INTAKE --> PARK: (intake.result == NO_BALL ||\n intake.result == ABORTED)\n && !timeEnough()

    %% ------------------------
    %% DRIVE_TO_SHOOT  (uses DriveController again)
    %% ------------------------
    state DRIVE_TO_SHOOT {
        DRIVE_SHOOT_CMD: drive.resetCycle();\ndrive.startCycle(GOTO_SHOOT_SPOT,\nSHOOT_SPOTS[shootIndex], timeout)
        DRIVE_SHOOT_CMD --> DRIVE_SHOOT_WAIT
        DRIVE_SHOOT_WAIT: wait until drive.state == DONE
    }

    DRIVE_TO_SHOOT --> SHOOT : drive.result == ARRIVED_OK

    DRIVE_TO_SHOOT --> PLAN_NEXT : drive.result == PATH_FAILED,  drive.result == ALIGN_FAILED,   drive.result == ABORTED, shootIndex++

    %% ------------------------
    %% SHOOT  (uses ShootController)
    %% ------------------------
    state SHOOT {
        SHOOT_CMD: shooter.resetCycle(), shooter.startCycle(targetShots,   currentDistance, shootTimeout)
        SHOOT_CMD --> SHOOT_WAIT
        SHOOT_WAIT: wait until shooter.state == DONE
    }

    SHOOT --> PLAN_NEXT : shooter.result == SUCCESS ballIndex++, shootIndex++

    SHOOT --> PLAN_NEXT : shooter.result == JAM_FAILED || shooter.result == ABORTED

    %% ------------------------
    %% PARK  (final drive cycle)
    %% ------------------------
    state PARK {
        PARK_CMD: drive.resetCycle();\ndrive.startCycle(GOTO_PARK,\nPARK_POS, timeout)
        PARK_CMD --> PARK_WAIT
        PARK_WAIT: wait until drive.state == DONE
    }

    PARK --> DONE

    state DONE {
        END: auto finished\n(teleop will take over)
    }

    DONE --> [*]

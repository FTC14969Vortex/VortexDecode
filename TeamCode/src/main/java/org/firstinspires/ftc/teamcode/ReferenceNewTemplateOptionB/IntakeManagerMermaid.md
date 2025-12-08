stateDiagram-v2
START --> IDLE
START: declare status, declare result enum(NONE, GOT_BALLS, NO_BALL, ABORTED)

    state IDLE {
        [*] --> IDLE_WAIT
        IDLE_WAIT: motors off    wait for startCycle()
    }

    IDLE --> RUNNING : startCycle(targetBalls, timeoutSec)  ballsCollected = 0, result = NONE

    state RUNNING {
        INTAKING: run intake motors  poll distance / beam sensor   count balls
    }

    %% Still collecting, time left, not enough balls
    RUNNING --> RUNNING: time < timeoutSec  ballsCollected < targetBalls

    %% Success: got >= 1 ball
    RUNNING --> DONE:  ballsCollected >= 1  ballsCollected >= targetBalls  stop intake, result = GOT_BALLS

    %% Partial success (optional case depending on your policy)
    RUNNING --> DONE: timeout && ballsCollected > 0    ballsCollected < targetBalls   stop intake, result = GOT_BALLS

    %% Hard failure: no ball at all
    RUNNING --> DONE:  timeout && ballsCollected == 0    stop intake, result = NO_BALL

    %% External cancel (e.g. time too short / auto period ending)
    RUNNING --> DONE: abortCycle()     stop intake,     result = ABORTED

    state DONE {
        DONE_WAIT: expose IntakeResult to GameManager
    }

    DONE --> IDLE: resetCycle()    state = IDLE, result = NONE

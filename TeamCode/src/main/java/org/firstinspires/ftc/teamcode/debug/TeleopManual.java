package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.utils.RobotOperations;

import org.firstinspires.ftc.teamcode.motion.MotionExecutor;

@TeleOp(name = "Teleop Manual 0.1", group = "Debug")
public class TeleopManual extends LinearOpMode {

    // Subsystems
    private BaseMotion baseMotion;
    private Intake intake;
    private FlyWheel flyWheel;
    private Kicker kicker;
    private Flipper flipper;
    private RobotOperations robotOps;

    // Driver thread state
    private volatile boolean driverThreadRunning = false;
    private Thread driverThread;

    // Debounce
    private final ElapsedTime debounce = new ElapsedTime();
    private boolean lastGp2A = false, lastGp2B = false, lastGp2X = false, lastGp2Y = false, lastGp2Rb = false;
    private static final double BTN_DEBOUNCE_SEC = 0.25;

    // Simple flags
    private boolean intakeOn = false;
    private boolean flywheelOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Teleop Manual...");
        telemetry.update();

// Initialize subsystems
        baseMotion = new BaseMotion();
        baseMotion.init(this); // creates MotionExecutor and Odometry etc. <source_id data="13" title="BaseMotion.java" />

        intake = new Intake();
        intake.init(this);
        intake.stopIntake(); // start safe <source_id data="16" title="Intake.java" />

        flyWheel = new FlyWheel();
        flyWheel.init(this); // uses RUN_WITHOUT_ENCODER by default and can start()/stop() <source_id data="12" title="FlyWheel.java" />

        kicker = new Kicker();
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE);

        flipper = new Flipper();
        flipper.init(hardwareMap);
        flipper.resetFlipper();

// Initialize RobotOperations (no CameraServo provided; alignment will degrade gracefully)
        robotOps = new RobotOperations();
        robotOps.init(
                baseMotion,
                flyWheel,
                intake,
                kicker,
                flipper,
                null, // CameraServo optional
                baseMotion.getMotionExecutor().getCoordinateTransformer(),
                this
        );

        telemetry.addLine("Initialization complete. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

// Start the driver thread for Gamepad1 driving
        startDriverThread();

        debounce.reset();
        while (opModeIsActive()) {
// Gamepad2 controls: intake, flywheel, near/far shoot
            handleGamepad2();

// Brief telemetry
            telemetry.addData("Drive Thread", driverThreadRunning ? "Running" : "Stopped");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            MotionExecutor.MotionResult last = robotOps.getLastMoveResult();
            telemetry.addData("Last Move", (last != null) ? (last.success ? "Success" : "Fail: " + last.failureReason) : "None");
            telemetry.update();

            sleep(20);
        }

// Clean shutdown
        stopDriverThread();
        robotOps.emergencyStop(); // stops motion and flywheel safely <source_id data="18" title="RobotOperations.java" />
        intake.stopIntake();
        flyWheel.stop();
    }

    private void handleGamepad2() {
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;
        boolean x = gamepad2.x;
        boolean y = gamepad2.y;
        boolean rb = gamepad2.right_bumper;

        double now = debounce.seconds();
        boolean debounceOk = now > BTN_DEBOUNCE_SEC;

// A: Shoot near (smart)
        if (a && !lastGp2A && debounceOk) {
            try {
// uses alliance-aware "shooting_near" move and shooting flow <source_id data="18" title="RobotOperations.java" />
                robotOps.executeSmartShooting("shooting_near", false, 4000);
            } catch (Exception e) {
                telemetry.addData("Shoot Near Error", e.getMessage());
            }
            debounce.reset();
        }
        lastGp2A = a;

// B: Shoot far (smart)
        if (b && !lastGp2B && debounceOk) {
            try {
                robotOps.executeSmartShooting("shooting_far", false, 4000);
            } catch (Exception e) {
                telemetry.addData("Shoot Far Error", e.getMessage());
            }
            debounce.reset();
        }
        lastGp2B = b;

// X: Toggle intake start/stop <source_id data="16" title="Intake.java" />
        if (x && !lastGp2X && debounceOk) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                intake.startIntake(); // full power <source_id data="16" title="Intake.java" />
            } else {
                intake.stopIntake();
            }
            debounce.reset();
        }
        lastGp2X = x;

// Y: Toggle flywheel start/stop (simple power-based control) <source_id data="12" title="FlyWheel.java" />
        if (y && !lastGp2Y && debounceOk) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
// convenience API uses a reasonable shooting power <source_id data="12" title="FlyWheel.java" />
                flyWheel.start();
            } else {
                flyWheel.stop();
            }
            debounce.reset();
        }
        lastGp2Y = y;

// RB: Quick stop everything (optional safety)
        if (rb && !lastGp2Rb && debounceOk) {
            robotOps.emergencyStop();
            intakeOn = false;
            flywheelOn = false;
            debounce.reset();
        }
        lastGp2Rb = rb;
    }

    private void startDriverThread() {
        if (driverThreadRunning) return;
        driverThreadRunning = true;

        driverThread = new Thread(() -> {
            try {
// NOTE: Uses same mapping as existing BaseMotion teleop helper:
// setMotorPowers(leftX, leftY, rightX) <source_id data="13" title="BaseMotion.java" />
                while (opModeIsActive() && !Thread.currentThread().isInterrupted() && driverThreadRunning) {
                    double leftX = gamepad1.left_stick_x; // strafe
                    double leftY = gamepad1.left_stick_y; // forward/backward
                    double rightX = gamepad1.right_stick_x; // rotation

// Apply driving command
                    baseMotion.setMotorPowers(leftX, leftY, rightX); // <source_id data="13" title="BaseMotion.java" />

                    try { Thread.sleep(20); } catch (InterruptedException ie) { Thread.currentThread().interrupt(); }
                }
            } catch (Exception e) {
// Best-effort: ensure stop if something fails
                baseMotion.stopRobot();
            }
        }, "Driver-Gamepad1-Thread");

        driverThread.setDaemon(true);
        driverThread.start();
    }

    private void stopDriverThread() {
        driverThreadRunning = false;
        if (driverThread != null) {
            try {
                driverThread.interrupt();
                driverThread.join(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                driverThread = null;
            }
        }
        baseMotion.stopRobot();
    }
}
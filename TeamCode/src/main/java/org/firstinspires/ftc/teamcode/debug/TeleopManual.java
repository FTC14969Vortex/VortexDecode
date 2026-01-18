package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.utils.RobotOperations;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;

@TeleOp(name = "Teleop Manual 0.2", group = "Teleop")
public class TeleopManual extends LinearOpMode {

    // Subsystems
    private BaseMotion baseMotion;
    private Intake intake;
    private FlyWheel flyWheel;
    private Kicker kicker;
    private Flipper flipper;
    private RobotOperations robotOps;

    // Driver thread
    private volatile boolean driverThreadRunning = false;
    private Thread driverThread;

    // Debounce
    private final ElapsedTime debounce = new ElapsedTime();
    private boolean lastA = false, lastB = false, lastX = false, lastY = false, lastRB = false;
    private static final double BTN_DEBOUNCE_SEC = 0.25;

    // State flags
    private boolean intakeOn = false;
    private boolean flywheelOn = false;

    // Optional: preset RPM for “quick shot” on B (no alignment)
    private static final double QUICK_SHOT_RPM = 1200.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Teleop Manual (No Navigation)...");
        telemetry.update();

// Initialize subsystems
        baseMotion = new BaseMotion();
        baseMotion.init(this); // creates MotionExecutor, Odometry, etc. <source_id data="13" title="BaseMotion.java" />

        intake = new Intake();
        intake.init(this);
        intake.stopIntake(); // safe baseline <source_id data="16" title="Intake.java" />

        flyWheel = new FlyWheel();
        flyWheel.init(this); // supports start()/stop() for simple power-based control <source_id data="12" title="FlyWheel.java" />

        kicker = new Kicker();
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE);

        flipper = new Flipper();
        flipper.init(hardwareMap);
        flipper.resetFlipper();

// RobotOperations without CameraServo (no navigation needed)
        robotOps = new RobotOperations();
        robotOps.init(
                baseMotion,
                flyWheel,
                intake,
                kicker,
                flipper,
                null, // CameraServo not provided
                baseMotion.getMotionExecutor().getCoordinateTransformer(),
                this
        );

        telemetry.addLine("Init complete. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

// Start driver thread for Gamepad 1 (translation + rotation)
        startDriverThread();

        debounce.reset();
        while (opModeIsActive()) {
            handleGamepad2();

// Minimal telemetry
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
        robotOps.emergencyStop(); // stops motion + flywheel + intake <source_id data="18" title="RobotOperations.java" />
        intake.stopIntake();
        flyWheel.stop();
    }

    // Gamepad 1 driving runs in a dedicated background thread
    private void startDriverThread() {
        if (driverThreadRunning) return;
        driverThreadRunning = true;

        driverThread = new Thread(() -> {
            try {
                while (opModeIsActive() && !Thread.currentThread().isInterrupted() && driverThreadRunning) {
// Mapping: BaseMotion.setMotorPowers(leftX, leftY, rightX) <source_id data="13" title="BaseMotion.java" />
                    double leftX = gamepad1.left_stick_x; // strafe
                    double leftY = gamepad1.left_stick_y; // forward/backward
                    double rightX = gamepad1.right_stick_x; // rotation

                    baseMotion.setMotorPowers(leftX, leftY, rightX); // teleop helper <source_id data="13" title="BaseMotion.java" />

                    try { Thread.sleep(20); } catch (InterruptedException ie) { Thread.currentThread().interrupt(); }
                }
            } catch (Exception e) {
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

    // Gamepad 2 actions: intake, flywheel, shoot here (no navigation)
    private void handleGamepad2() {
        boolean a = gamepad2.a; // Shoot here with alignment (if available) <source_id data="18" title="RobotOperations.java" />
        boolean b = gamepad2.b; // Quick shot here, no alignment <source_id data="18" title="RobotOperations.java" />
        boolean x = gamepad2.x; // Intake toggle <source_id data="16" title="Intake.java" />
        boolean y = gamepad2.y; // Flywheel toggle <source_id data="12" title="FlyWheel.java" />
        boolean rb = gamepad2.right_bumper; // Emergency stop (optional safety) <source_id data="18" title="RobotOperations.java" />

        double now = debounce.seconds();
        boolean debounceOk = now > BTN_DEBOUNCE_SEC;

// A: Shoot HERE with alignment (no navigation).
// When CameraServo is not provided, RobotOperations falls back to odometry distance and does alignment internally.
        if (a && !lastA && debounceOk) {
            try {
                robotOps.shoot(true); // useCameraServo=true as a hint; will gracefully fallback if none <source_id data="18" title="RobotOperations.java" />
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                telemetry.addData("Shoot(A)", "Interrupted");
            } catch (Exception e) {
                telemetry.addData("Shoot(A) Error", e.getMessage());
            }
            debounce.reset();
        }
        lastA = a;

// B: Quick shot HERE without alignment (fast fire, no turning).
        if (b && !lastB && debounceOk) {
            try {
// Full control API: shoot(targetVelocity, alignToShootingAngle=false, useCameraServo=false) <source_id data="18" title="RobotOperations.java" />
                robotOps.shoot(QUICK_SHOT_RPM, false, false);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                telemetry.addData("Shoot(B)", "Interrupted");
            } catch (Exception e) {
                telemetry.addData("Shoot(B) Error", e.getMessage());
            }
            debounce.reset();
        }
        lastB = b;

// X: Toggle intake ON/OFF <source_id data="16" title="Intake.java" />
        if (x && !lastX && debounceOk) {
            intakeOn = !intakeOn;
            if (intakeOn) intake.startIntake(); else intake.stopIntake();
            debounce.reset();
        }
        lastX = x;

// Y: Toggle flywheel start/stop (simple power-based) <source_id data="12" title="FlyWheel.java" />
        if (y && !lastY && debounceOk) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
                flyWheel.start(); // convenience power-based on <source_id data="12" title="FlyWheel.java" />
            } else {
                flyWheel.stop();
            }
            debounce.reset();
        }
        lastY = y;

// RB: Emergency stop – halts motion + flywheel + intake immediately <source_id data="18" title="RobotOperations.java" />
        if (rb && !lastRB && debounceOk) {
            robotOps.emergencyStop();
            intakeOn = false;
            flywheelOn = false;
            debounce.reset();
        }
        lastRB = rb;
    }
}
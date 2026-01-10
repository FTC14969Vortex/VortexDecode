package org.firstinspires.ftc.teamcode.debug;

//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//subsytems
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;


import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.utils.RobotOperations;
import org.firstinspires.ftc.teamcode.utils.RobotUtil;
import org.firstinspires.ftc.teamcode.vision.CameraServo;


@TeleOp(name = "TeleOpBlueNearManual 0.03", group = "TeleOp")

public class TeleopBlueManual extends LinearOpMode {
    BaseMotion baseMotion;
    Chassis chassis;
    Intake intake;
    Flipper flipper;
    Kicker kicker;
    FlyWheel flyWheel;
    CameraServo cameraServo;
    private RobotOperations robotOperations;
    private volatile boolean threadIsRunning = true;

    // ========== STATE TRACKING ==========
    private ElapsedTime buttonTimer = new ElapsedTime();
    private boolean intakeOn = false;
    private String lastOperation = "None";
    private boolean isMovingToShoot = false;  // Track if we're in a shooting movement

    // ========== BUTTON DEBOUNCING ==========
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1Y = false;
    private boolean lastGamepad2A = false;
    private boolean lastGamepad2B = false;
    private boolean lastGamepad2X = false;
    private boolean lastGamepad2Y = false;
    private boolean lastGamepad2RightBumper = false;

    // ========== CONTROL PARAMETERS ==========
    private static final double DRIVE_SPEED_MULTIPLIER = 1.0;  // Full speed for translation
    private static final double ROTATION_SPEED_MULTIPLIER = 0.3;  // Reduced speed for rotation precision
    private static final double BUTTON_DEBOUNCE_TIME = 0.3;    // seconds


    long maxLoopTimeout = 2000;
    private Thread driveThread;

    @Override
    public void runOpMode() throws InterruptedException {

        // ========== SUBSYSTEMS ==========

        cameraServo = new CameraServo();

        // Initialize subsystems
        baseMotion = new BaseMotion();
        baseMotion.init(this);

        flyWheel = new FlyWheel();
        flyWheel.init(this);

        intake = new Intake();
        intake.init(this);
        intake.stopIntake();

        kicker = new Kicker();
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE); // Start in intake position

        flipper = new Flipper();
        flipper.init(hardwareMap);
        flipper.resetFlipper();

        // Initialize CameraServo with full motion integration for proper pose-based aiming
        cameraServo.init(
                hardwareMap,
                baseMotion.getMotionExecutor().getMotionState().getOdometryManager(),
                baseMotion.getMotionExecutor().getCoordinateTransformer(),
                baseMotion.getMotionExecutor()
        );
        cameraServo.moveToCenter(); // Keep servo at center position for this auto
        cameraServo.setAutoOdometryCorrection(false); // Disable autocorrection for pure odometry-based calculation
        cameraServo.update();

        // Initialize RobotOperations utility
        robotOperations = new RobotOperations();
        robotOperations.init(baseMotion, flyWheel, intake, kicker, flipper, cameraServo,
                baseMotion.getMotionExecutor().getCoordinateTransformer(), this);

        // Define and start the drive thread
        driveThread = new Thread(new DriveTask());

        waitForStart();

        driveThread.start(); // Start the concurrent task


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            gamepad2Controls();


            // Clean up the thread
            threadIsRunning = false;
            sleep(2000);
            driveThread.interrupt();
            try {
                driveThread.join();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }
    }

        private class DriveTask implements Runnable {

            @Override
            public void run() {
                while (threadIsRunning && !Thread.currentThread().isInterrupted()) {


                    // Read gamepad input and set drive motor power
                    float axial = -gamepad1.left_stick_y;
                    float lateral = -gamepad1.left_stick_x;
                    float yaw = -gamepad1.right_stick_x; // Note: positive yaw is clockwise, previously was negative
                    RobotUtil.setMotorPower(chassis.frontLeftDrive, chassis.backLeftDrive,
                            chassis.frontRightDrive, chassis.backRightDrive,
                            axial, lateral, yaw);

                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }
            }
        }

        private void gamepad2Controls() throws InterruptedException{

            if (gamepad2.x){
                robotOperations.prepareForIntake();
                intake.startIntake();
            }else if (gamepad2.left_bumper) {
                robotOperations.prepareForShooting();
            } else if (gamepad2.right_bumper) {
                robotOperations.shoot();
            }
        }

        }

    }


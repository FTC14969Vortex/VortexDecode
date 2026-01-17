package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.OdometryManager;
import org.firstinspires.ftc.teamcode.motion.CoordinateTransformer;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Locale;

/**
 * UNIFIED Camera Servo Controller for FTC Robot
 *
 * Single-file solution that combines:
 * 1. AprilTag detection (preset blue or red tag)
 * 2. Distance measurement from detected tags
 * 3. Flywheel velocity calculation based on distance
 * 4. Optional odometry correction using tag positions
 * 5. Camera servo control for tag targeting and search
 *
 * Hardware: GoBilda 5-turn servo + camera with AprilTag processor
 * - Servo Range: 1620° total (-810° to +810° from center, 4.5 turns max)
 * - Servo Speed: 600°/sec
 * - AprilTag Detection: Blue GOAL (Tag 20), Red GOAL (Tag 24)
 *
 * Usage:
 * 1. cameraServo.init(hardwareMap, aprilTagProcessor)
 * 2. cameraServo.setTargetTag(20) // Set blue or red tag
 * 3. cameraServo.update() // Call in robot loop
 * 4. double velocity = cameraServo.getFlywheelVelocity()
 * 5. FieldPose corrected = cameraServo.getCorrectedPose(odometry)
 */
public class CameraServo {

    // ========== SERVO CONFIGURATION ==========

    /** Camera servo hardware name in configuration */
    private static final String CAMERA_SERVO_NAME = "cameraServo";

    /** Total servo range in degrees (GoBilda 1-turn servo: 1 turns = 300°) */
    private static final double SERVO_RANGE_DEGREES = 300.0;

    /** Maximum servo angle from center (+810 degrees) - HARDWARE LIMIT */
    private static final double SERVO_MAX_ANGLE = SERVO_RANGE_DEGREES/2;

    /** Minimum servo angle from center (-810 degrees) - HARDWARE LIMIT */
    private static final double SERVO_MIN_ANGLE = -SERVO_MAX_ANGLE;

    /** Maximum USAGE angle from center (+180 degrees) - PRACTICAL LIMIT */
    private static final double SERVO_USAGE_MAX_ANGLE = 130;

    /** Minimum USAGE angle from center (-180 degrees) - PRACTICAL LIMIT */
    private static final double SERVO_USAGE_MIN_ANGLE = -130;


    /** Minimum servo position (0.0) */
    private static final double SERVO_MIN_POSITION = 0.0;

    /** Maximum servo position (1.0) */
    private static final double SERVO_MAX_POSITION = 1.0;

    /** Maximum servo speed in degrees per second */
    private static final double SERVO_MAX_SPEED = 600.0;  // GoBilda spec

    /** Servo position tolerance for "at target" detection */
    private static final double SERVO_POSITION_TOLERANCE = 2.0;  // degrees

    /**
     *   * COORDINATE SYSTEM CONVENTION:     *
     * - From driver's perspective behind robot:
     *   - Positive servo input: camera rotates LEFT (counterclockwise)
     *   - Negative servo input: camera rotates RIGHT (clockwise)
     *
     * If physical servo behavior is opposite, set this to false
     */
    private static final boolean SERVO_DIRECTION_CCW = true;

    // ========== SEARCH CONFIGURATION ==========

    /** Search range from center position (±degrees) */
    private static final double SEARCH_RANGE_DEGREES = 20.0;

    /** Servo search step size (degrees) */
    private static final double SEARCH_STEP_DEGREES = 10.0;

    /** Time to wait at each search position (milliseconds) */
    private static final long SEARCH_DWELL_TIME_MS = 200;

    /** Update frequency (Hz) - CameraServo runs at 5 Hz */
    private static final double UPDATE_FREQUENCY_HZ = 5.0;

    /** Update interval (milliseconds) - 200ms for 5 Hz */
    private static final long UPDATE_INTERVAL_MS = (long)(1000.0 / UPDATE_FREQUENCY_HZ);

    // ========== APRILTAG CONFIGURATION ==========

    /**
     * Camera offset from robot center (inches, degrees) - Using BACK_CAMERA from RobotConstants
     * NOTE: Named "back camera" but actually FORWARD-FACING (aligned with robot heading at servo=0°)
     * When servo is at 0°, camera faces +X direction (same as robot forward)
     */
    private static final double CAMERA_OFFSET_X = RobotConstants.BACK_CAMERA.x;    // Forward/backward from robot center
    private static final double CAMERA_OFFSET_Y = RobotConstants.BACK_CAMERA.y;    // Left/right from robot center
    private static final double CAMERA_OFFSET_HEADING = RobotConstants.BACK_CAMERA.yaw; // Rotation offset from robot heading (camera facing )

    // Servo offset from robot center (inches)
    private static final double SERVO_OFFSET_X = RobotConstants.BACK_CAMERA_SERVO.x;
    private static final double SERVO_OFFSET_Y = RobotConstants.BACK_CAMERA_SERVO.y;

    // camera offset from servo center (inches)
    private static final double CAMERA_SERVO_OFFSET_X = CAMERA_OFFSET_X - SERVO_OFFSET_X;
    private static final double CAMERA_SERVO_OFFSET_Y = CAMERA_OFFSET_Y - SERVO_OFFSET_Y;

    // ========== FLYWHEEL VELOCITY PARAMETERS ==========

    /** Linear flywheel velocity dependence on distance */
    private static final double FLYWHEEL_VELOCITY_SLOPE = 6.51;      // RPM per inch
    private static final double FLYWHEEL_VELOCITY_INTERCEPT = 800.0; // Base RPM

    /** Minimum flywheel velocity (RPM) */
    private static final double MIN_FLYWHEEL_VELOCITY = 799;

    /** Maximum flywheel velocity (RPM) */
    private static final double MAX_FLYWHEEL_VELOCITY = 2000;

    // ========== POSE FUSION PARAMETERS ==========

    /** Vision weight for pose fusion (0.0 = odometry only, 1.0 = vision only) */
    private static final double VISION_FUSION_WEIGHT = 0.1;

    /** Maximum distance for pose correction (inches) */
    private static final double MAX_CORRECTION_DISTANCE = 80.0;
    private static final double MAX_CORRECTION_Angle = 90.0;

    // ========== DETECTION DISTANCE LIMITS ==========

    /** Minimum reliable detection distance (inches) */
    private static final double MIN_DETECTION_DISTANCE = 40.0;

    /** Maximum reliable detection distance (inches) */
    private static final double MAX_DETECTION_DISTANCE = 120.0;

    // ========== AUTOMATIC ODOMETRY CORRECTION ==========

    /** Enable automatic odometry correction when AprilTags are detected */
    private boolean autoOdometryCorrection = false;
    // ========== SERVO MOVEMENT CONTROL ==========

    /** Enable servo movement (search, tracking, etc.) - when false, servo stays at center */
    private boolean servoMovementEnabled = true;  // Default enabled for backward compatibility

    // ========== HARDWARE ==========

    private Servo panServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private OdometryManager odometryManager;
    private CoordinateTransformer coordinateTransformer;
    private MotionExecutor motionExecutor;
    private ElapsedTime timer;
    private ElapsedTime searchTimer;

    // ========== STATE TRACKING ==========

    private double currentAngle = 0.0;      // Current servo angle
    private double targetAngle = 0.0;       // Target servo angle in degrees based on prediction
    private boolean isInitialized = false;
    private boolean isSearching = false;
    private int searchStep = 0;

    // Vision state
    private double lastFlywheelVelocity = MIN_FLYWHEEL_VELOCITY;
    private long lastDetectionTime = 0;
    private int lastDetectedTagId = -1;
    private double lastDetectedDistance = 0.0;
    private AprilTagDetection lastDetection = null;

    private double lastShootingAngle = 0; // robot rotation angle for shooting, in degree

    // Additional detection data for diagnostics
    private double lastDetectedBearing = 0.0;   // bearing angle to tag (degrees)
    private double lastDetectedYaw = 0.0;       // yaw angle of tag (degrees)
    private double lastDetectedElevation = 0.0; // elevation angle to tag (degrees)
    private double lastFtcPoseX = 0.0;          // ftcPose.x from last detection (camera frame lateral)
    private double lastFtcPoseY = 0.0;          // ftcPose.y from last detection (camera frame forward)


    // Enhanced diagnostics

    private FieldPose lastCameraPosition = new FieldPose(0, 0, 0);  // camera position when last detection occurred
    private FieldPose lastServoPosition = new FieldPose(0, 0, 0);   // servo position in field coordinates when last detection occurred

    private boolean hasInitializedPosition = false;

    // Target tag selection
    private int targetTagId = 20; // Default to blue GOAL (Tag 20)

    // Threading state
    private Thread updateThread = null;
    private volatile boolean isThreadRunning = false;
    private long lastUpdateTime = 0;

    // ========== INITIALIZATION ==========

    /**
     * Initializes the AprilTag servo system
     *
     * @param hardwareMap Robot hardware map
     * @param odometryManager Odometry manager for automatic pose corrections (optional)
     * @param coordinateTransformer Coordinate transformer for pose calculations (optional)
     * @param motionExecutor Motion executor for robot rotation during shooting alignment (optional)
     */
    public void init(HardwareMap hardwareMap, OdometryManager odometryManager, CoordinateTransformer coordinateTransformer, MotionExecutor motionExecutor) {
        try {
            // Initialize servo hardware
            panServo = hardwareMap.get(Servo.class, CAMERA_SERVO_NAME);
            // Initialize vision system internally
            initVisionSystem(hardwareMap);
            this.odometryManager = odometryManager;
            this.coordinateTransformer = coordinateTransformer;
            this.motionExecutor = motionExecutor;
            timer = new ElapsedTime();
            searchTimer = new ElapsedTime();
            timer.reset();

            // Set to center position on init
            moveToCenter();
            update();

            isInitialized = true;

        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize AprilTag servo: " + e.getMessage(), e);
        }
    }

    /**
     * Initializes the AprilTag servo system (without automatic odometry correction)
     *
     * @param hardwareMap Robot hardware map
     */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, null, null, null);
    }
    /**
     * Initialize vision system internally
     *
     * @param hardwareMap Robot hardware map
     */
    private void initVisionSystem(HardwareMap hardwareMap) {
        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // ========== TARGET TAG SELECTION ==========

    /**
     * Sets the target AprilTag for detection
     *
     * @param tagId Target AprilTag ID (20 for blue GOAL, 24 for red GOAL)
     */
    public void setTargetTag(int tagId) {
        if (tagId == 20 || tagId == 24) {
            targetTagId = tagId;
        } else {
            throw new IllegalArgumentException("Target tag must be 20 (blue GOAL) or 24 (red GOAL)");
        }
    }

    /**
     * Gets the current target tag ID
     *
     * @return Current target AprilTag ID
     */
    public int getTargetTag() {
        return targetTagId;
    }

    // ========== AUTOMATIC ODOMETRY CORRECTION CONFIGURATION ==========

    /**
     * Enables or disables automatic odometry correction
     *
     * @param enabled True to enable automatic corrections, false to disable
     */
    public void setAutoOdometryCorrection(boolean enabled) {
        this.autoOdometryCorrection = enabled;
    }
    /**
     * Enable or disable servo movement (search, tracking, etc.)
     * When disabled, servo stays at center position and only performs passive detection
     *
     * @param enabled True to enable servo movement, false to keep stationary at center
     */
    public void setServoMovementEnabled(boolean enabled) {
        this.servoMovementEnabled = enabled;
        if (!enabled) {
            // When disabling movement, stop any active search and move to center
            stopSearch();
            moveToCenter();
        }
    }

    /**
     * Checks if servo movement is enabled
     *
     * @return True if servo movement is enabled
     */
    public boolean isServoMovementEnabled() {
        return servoMovementEnabled;
    }

    /**
     * Checks if automatic odometry correction is enabled
     *
     * @return True if automatic corrections are enabled
     */
    public boolean isAutoOdometryCorrectionEnabled() {
        return autoOdometryCorrection;
    }

    // ========== MAIN UPDATE LOOP ==========

    /**
     * Main update method - call this in robot loop
     * Camera always faces predicted AprilTag position and searches when needed
     */
    public void update() {
        if (!isInitialized) return;

        motionExecutor.updateState(); // Update motion executor state, coordinates

        // Always process AprilTag detections for passive detection
        processAprilTagDetections();

        // Skip all servo movement if movement is disabled (stationary mode)
        if (!servoMovementEnabled) {
            return;
        }

        // Get current robot pose for prediction
        if (motionExecutor != null) {
            Pose2D referencePointPose2D = motionExecutor.getMotionState().getCurrentPose();
            FieldPose currentPose = new FieldPose(referencePointPose2D.getX(DistanceUnit.INCH),
                    referencePointPose2D.getY(DistanceUnit.INCH),
                    referencePointPose2D.getHeading(AngleUnit.DEGREES));

            aimAtTag(currentPose, targetTagId);
        }

        // Update search pattern if active
        if (isSearching) {
            updateSearch(targetAngle);
        }

        // Update servo position with velocity limiting
        updateServoPosition();



    }

    // ========== THREADING CONTROL ==========

    /**
     * Starts the CameraServo as a background thread running at 5 Hz
     * Thread-safe operation with automatic timing control
     */
    public synchronized void startThread() {
        if (isThreadRunning || !isInitialized) {
            return; // Already running or not initialized
        }

        isThreadRunning = true;
        updateThread = new Thread(this::threadUpdateLoop, "CameraServo-Thread");
        updateThread.setDaemon(true); // Don't prevent JVM shutdown
        updateThread.start();
    }

    /**
     * Stops the CameraServo background thread
     * Thread-safe operation with proper cleanup
     */
    public synchronized void stopThread() {
        if (!isThreadRunning) {
            return; // Already stopped
        }

        isThreadRunning = false;

        if (updateThread != null) {
            try {
                updateThread.interrupt();
                updateThread.join(500); // Wait up to 500ms for clean shutdown
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Restore interrupt status
            }
            updateThread = null;
        }
    }

    /**
     * Main thread loop - runs at exactly 5 Hz (200ms intervals)
     * Handles timing automatically and calls update() at precise intervals
     */
    private void threadUpdateLoop() {
        lastUpdateTime = System.currentTimeMillis();

        while (isThreadRunning && !Thread.currentThread().isInterrupted()) {
            try {
                // Calculate time since last update
                long currentTime = System.currentTimeMillis();
                long timeSinceLastUpdate = currentTime - lastUpdateTime;

                // Only update if enough time has passed (5 Hz = 200ms)
                if (timeSinceLastUpdate >= UPDATE_INTERVAL_MS) {
                    update(); // Call the main update method
                    lastUpdateTime = currentTime;
                }

                // Sleep for a short time to prevent busy waiting
                Thread.sleep(20); // 10ms sleep for responsive shutdown

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break; // Exit loop on interrupt
            } catch (Exception e) {
                // Log error but continue running
                System.err.println("CameraServo thread error: " + e.getMessage());
            }
        }
    }

    /**
     * Checks if the CameraServo thread is currently running
     * @return true if thread is active, false otherwise
     */
    public boolean isThreadRunning() {
        return isThreadRunning;
    }

    // ========== APRILTAG DETECTION ==========

    /**
     * Processes AprilTag detections and updates flywheel velocity
     * Implements search behavior when tag not detected but within reliable range
     */
    private void processAprilTagDetections() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Find target tag detection
        AprilTagDetection targetDetection = null;
        if (detections != null && !detections.isEmpty()) {
            targetDetection = findTargetTag(detections);
        }

        double predictedDistance = 0.0;
        boolean withinReliableRange = false;

        if (coordinateTransformer != null) {
            Pose2D robotCenterPose2D = coordinateTransformer.getCurrentRobotCenterPose();
            FieldPose robotCenterPose = new FieldPose(
                    robotCenterPose2D.getX(DistanceUnit.INCH),
                    robotCenterPose2D.getY(DistanceUnit.INCH),
                    robotCenterPose2D.getHeading(AngleUnit.DEGREES)
            );

            FieldPose tagPose = getAprilTagPosition(targetTagId);
            if (tagPose != null) {
                // Calculate distance from camera to tag (not robot center to tag)
                // Use robot center position directly for flywheel velocity calculation
                predictedDistance = calculateDistance(robotCenterPose, tagPose);
                withinReliableRange = (predictedDistance >= MIN_DETECTION_DISTANCE &&
                        predictedDistance <= MAX_DETECTION_DISTANCE);
            }
        }

        if (targetDetection != null) {
            // Tag detected - update state and stop searching
            lastDetectionTime = System.currentTimeMillis();
            lastDetectedTagId = targetDetection.id;
            lastDetectedDistance = calculateDistance(targetDetection);
            lastDetection = targetDetection; // Store detection for vision-based position calculation

            // Calculate and update flywheel velocity
            lastFlywheelVelocity = calculateFlywheelVelocity(lastDetectedDistance);

            // Shooting angle is now calculated in getVisionPoseFromDetection() using proper odometry

            // Capture additional detection data for diagnostics
            lastDetectedBearing = targetDetection.ftcPose.bearing;
            lastDetectedYaw = targetDetection.ftcPose.yaw;
            lastDetectedElevation = targetDetection.ftcPose.elevation;

            // Always populate enhanced diagnostics (camera/servo positions) when we have a detection
            // This ensures diagnostics are available regardless of auto correction settings
            getVisionPoseFromDetection(targetDetection);

            boolean detectionWithinReliableRange = (lastDetectedDistance >= MIN_DETECTION_DISTANCE &&
                    lastDetectedDistance <= MAX_DETECTION_DISTANCE);

            if (detectionWithinReliableRange) {
                performAutomaticOdometryCorrection(targetDetection);
            }

            // Stop searching since we found the target
            if (isSearching) {
                stopSearch();
            }
        } else {
            // Tag not detected calculate flywheel velocity from predicted odometry
            lastFlywheelVelocity = calculateFlywheelVelocity(predictedDistance);

            // Calculate shooting angle using odometry when no detection
            if (coordinateTransformer != null) {
                Pose2D robotCenterPose2D = coordinateTransformer.getCurrentRobotCenterPose();
                FieldPose tagPose = getAprilTagPosition(targetTagId);
                if (tagPose != null) {
                    // Calculate angle from robot center to tag
                    double robotX = robotCenterPose2D.getX(DistanceUnit.INCH);
                    double robotY = robotCenterPose2D.getY(DistanceUnit.INCH);
                    double robotHeading = robotCenterPose2D.getHeading(AngleUnit.DEGREES);

                    double angleToTag = Math.toDegrees(Math.atan2(tagPose.y - robotY, tagPose.x - robotX));
                    lastShootingAngle = normalizeAngle(angleToTag - robotHeading - 180.0);
                }
            }
            if (!isSearching) {
                // Start search pattern around current predicted angle
                startSearch(targetAngle);
            }
        }
    }

    /**
     * Finds the preset target AprilTag from available detections
     * Only looks for the specific tag ID set by setTargetTag()
     */
    private AprilTagDetection findTargetTag(List<AprilTagDetection> detections) {
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                return detection; // Found the target tag
            }
        }
        return null; // Target tag not found
    }

    /**
     * Calculates distance from camera to detected AprilTag
     * Uses detection.range when available (direct tag-to-camera distance),
     * otherwise calculates using tag position and camera position
     */
    private double calculateDistance(AprilTagDetection detection) {
        // PREFERRED: Use detection.ftcPose.range if available (direct tag-to-camera distance from SDK)
        if (detection.ftcPose != null && detection.ftcPose.range > 0) {
            // OLD: return detection.ftcPose.range; // This was camera-to-tag distance
            // NEW: Calculate robot center-to-tag distance for accurate flywheel velocity
            FieldPose tagPose = getAprilTagPosition(detection.id);
            if (tagPose != null && lastDetection != null) {
                // Get robot center position from vision calculation
                FieldPose visionRobotPose = getVisionPoseFromDetection(detection);
                if (visionRobotPose != null) {
                    return calculateDistance(tagPose, visionRobotPose);
                }
            }
            // Fallback to camera-to-tag distance if vision calculation fails
            return detection.ftcPose.range;
        }

        // LAST RESORT: Calculate tag-to-camera distance using field positions
        FieldPose tagPose = getAprilTagPosition(detection.id);
        if (tagPose != null && coordinateTransformer != null) {
            // Get robot center position from motion system
            Pose2D robotCenter = coordinateTransformer.getCurrentRobotCenterPose();
            FieldPose robotPose = new FieldPose(
                    robotCenter.getX(DistanceUnit.INCH),
                    robotCenter.getY(DistanceUnit.INCH),
                    robotCenter.getHeading(AngleUnit.DEGREES)
            );

            // Calculate camera position using camera offset
            // Use robot center position directly (not camera position)
            return calculateDistance(tagPose, robotPose);
        }

        // No reliable distance available
        return 0.0;
    }

    private double calculateDistance(FieldPose pose1, FieldPose pose2) {
        // Input validation to prevent mathematical errors
        if (pose1 == null || pose2 == null) {
            return 0.0;
        }

        double dx = pose1.x - pose2.x;
        double dy = pose1.y - pose2.y;
        double distanceSquared = dx * dx + dy * dy;

        return Math.sqrt(distanceSquared);
    }



    // ========== FLYWHEEL VELOCITY CALCULATION ==========

    public double getFlywheelVelocity() {
        return lastFlywheelVelocity;
    }
    public double getShootingAngle() {return lastShootingAngle;}

    private double calculateFlywheelVelocity(double distance) {
        if (distance <= 0) {
            return MIN_FLYWHEEL_VELOCITY;
        }

        double velocity = FLYWHEEL_VELOCITY_SLOPE * distance + FLYWHEEL_VELOCITY_INTERCEPT;
        return Math.max(MIN_FLYWHEEL_VELOCITY, Math.min(MAX_FLYWHEEL_VELOCITY, velocity));
    }

    // ========== ODOMETRY CORRECTION ==========

    /**
     * Gets pose-corrected robot position using AprilTag detection
     *
     * @param odometryPose Current robot pose from odometry
     * @return Corrected pose (fusion of odometry + vision), or original if no correction
     */
    public FieldPose getCorrectedPose(AprilTagDetection detection) {

        if ( odometryManager == null | detection == null) {
            return null;
        }

        // Get current odometry pose (reference point coordinates directly)
        FieldPose currentOdometryPose;
        Pose2D refPointPose2D = odometryManager.getCurrentPose();
        if (refPointPose2D != null) {
            currentOdometryPose = new FieldPose(refPointPose2D.getX(DistanceUnit.INCH),
                    refPointPose2D.getY(DistanceUnit.INCH),
                    refPointPose2D.getHeading(AngleUnit.DEGREES));
        } else {
            return null; // No current odometry pose available
        }

        // Get vision-based reference point pose from detection
        FieldPose visionPose = getVisionPoseFromDetection(detection);
        if (visionPose == null) {
            return null; // Vision pose not available
        }

        // Check if correction distance is reasonable (ignore on first detection)
        double correctionDistance = calculateDistance(currentOdometryPose, visionPose);
        if (!hasInitializedPosition) {
            // First detection - accept any distance to establish initial position
            hasInitializedPosition = true;
        } else {
            if (correctionDistance > MAX_CORRECTION_DISTANCE) {
                return null; // Subsequent detections - reject if correction too large
            }

            double correctionAngle = Math.abs(visionPose.heading - currentOdometryPose.heading);
            if (correctionAngle > MAX_CORRECTION_Angle) {
                return null; // Subsequent detections - reject if correction too large
            }
        }

        // Fuse odometry and vision poses (both now in reference point coordinates)
        // This eliminates coordinate frame mismatch and provides mathematically correct fusion
        return  fusePoses(currentOdometryPose, visionPose, VISION_FUSION_WEIGHT);

    }

    /**
     * Calculates robot pose from camera detection using mathematically correct transformation
     * tagpose - camera pose - servo pose - robot pose
     * detection.ftc.pose.x: side, detection.ftc.pose.y: forward -- needed to be reversed for robot coordinate system
     */
    private FieldPose getVisionPoseFromDetection(AprilTagDetection detection) {
        // 1. Known tag pose in field
        FieldPose tagPose = getAprilTagPosition(detection.id);
        if (tagPose == null) return null;

        // 2. Extract FTC measurements

        // Tag in camera frame pos with robot coordinate convention: x-forward, y-side
        double x_camTag = detection.ftcPose.y;
        double y_camTag = -detection.ftcPose.x;

        // Cache raw ftcPose values for telemetry (SDK camera-frame axes)
        lastFtcPoseX = detection.ftcPose.x;
        lastFtcPoseY = detection.ftcPose.y;

        //ftcpose.yaw: camera + at tag left side, - at tag right side

        // 3. camera heading
        //camera heading in field, looking from the front of camera to back, tag - view of cam
        // camera heading in field, looking from the back of camera to front
        double camFieldHeading = tagPose.heading - detection.ftcPose.yaw;


        // 4. Robot heading
        //camera heading in robot, looking from back of camera to front
        double camToRobotHeading = currentAngle + CAMERA_OFFSET_HEADING;
        double robotHeading = normalizeAngle(camFieldHeading - camToRobotHeading);

        // 5. camera position in field frame
        double cosC = Math.cos(Math.toRadians(camFieldHeading));
        double sinC = Math.sin(Math.toRadians(camFieldHeading));
        double camFieldX = tagPose.x - (x_camTag * cosC - y_camTag * sinC);
        double camFieldY = tagPose.y - (x_camTag * sinC + y_camTag * cosC);

        // 6. Servo positon in field: subtract camera offset to get servo pos in field
        double servoFieldX = camFieldX - (CAMERA_SERVO_OFFSET_X * cosC - CAMERA_SERVO_OFFSET_Y * sinC);
        double servoFieldY = camFieldY - (CAMERA_SERVO_OFFSET_X * sinC + CAMERA_SERVO_OFFSET_Y * cosC);

        // 7. Robot center position in field:  subtract servo offset to get robot center in field
        // NOTE: servo offset is not rotated with servo
        double cosR = Math.cos(Math.toRadians(robotHeading));
        double sinR = Math.sin(Math.toRadians(robotHeading));
        double robotX = servoFieldX - (SERVO_OFFSET_X * cosR - SERVO_OFFSET_Y * sinR);
        double robotY = servoFieldY - (SERVO_OFFSET_X * sinR + SERVO_OFFSET_Y * cosR);

        // 7. Convert robot center → reference point
        Pose2D robotCenterPose = new Pose2D(
                DistanceUnit.INCH,
                robotX,
                robotY,
                AngleUnit.DEGREES,
                robotHeading
        );

        // Store enhanced diagnostics from the vision calculation chain
        lastCameraPosition = new FieldPose(camFieldX, camFieldY, camFieldHeading);
        lastServoPosition = new FieldPose(servoFieldX, servoFieldY, robotHeading); // servo heading = robot heading

        // Calculate shooting angle
        // This is the angle the robot needs to rotate so its back faces the tag center
        double dx = tagPose.x - robotX;
        double dy = tagPose.y - robotY;
        double angleToTag = Math.toDegrees(Math.atan2(dy, dx));
      
        double ShootingAngle_yaw = normalizeAngle(angleToTag - robotHeading - 180.0); // from coordinates + ftc.pose.yaw        
        double ShootingAngle_bearing =  detection.ftcPose.bearing; // camera bearing
        
        lastShootingAngle = (ShootingAngle_yaw + ShootingAngle_bearing)/2;

        return CoordinateTransformer.convertRobotCenterToReferencePoint(robotCenterPose);

    }

    /**
     * Fuses two poses using weighted average
     * CRITICAL: Uses proper circular averaging for angles to handle wraparound correctly
     */
    private FieldPose fusePoses(FieldPose pose1, FieldPose pose2, double weight2) {
        // Input validation
        if (pose1 == null) return pose2;
        if (pose2 == null) return pose1;
        if (weight2 < 0.0 || weight2 > 1.0) {
            weight2 = Math.max(0.0, Math.min(1.0, weight2)); // Clamp to valid range
        }

        double weight1 = 1.0 - weight2;

        // Linear interpolation for position (X, Y) - this is correct
        double fusedX = pose1.x * weight1 + pose2.x * weight2;
        double fusedY = pose1.y * weight1 + pose2.y * weight2;

        // PROPER circular interpolation for heading using shortest path
        double angleDiff = pose2.heading - pose1.heading;

        // Normalize the difference to [-180, 180] to get shortest path
        while (angleDiff > 180) angleDiff -= 360;
        while (angleDiff < -180) angleDiff += 360;

        // Interpolate along the shortest path
        double fusedHeading = pose1.heading + (angleDiff * weight2);

        return new FieldPose(fusedX, fusedY, normalizeAngle(fusedHeading));
    }



    /**
     * Performs automatic odometry correction using AprilTag detection
     * Fuses current odometry pose with vision pose instead of overwriting
     *
     * @param detection AprilTag detection to use for correction
     */
    private void performAutomaticOdometryCorrection(AprilTagDetection detection) {
        // Check if automatic correction is enabled and odometry manager is available
        if (!autoOdometryCorrection || odometryManager == null) {
            return;
        }

        FieldPose fusedPose = getCorrectedPose(detection);
        // Apply the fused correction to odometry
        try {
            // Convert FieldPose to Pose2D for odometry manager
            Pose2D correctionPose = new Pose2D(
                    DistanceUnit.INCH, fusedPose.x, fusedPose.y,
                    AngleUnit.DEGREES, fusedPose.heading
            );

            // use motionExecutor  
            motionExecutor.resetToFieldOrigin(correctionPose);
            
          //  odometryManager.resetToPose(correctionPose);

        } catch (Exception e) {
            // Silently handle errors to avoid disrupting robot operation
            // In production, this could log to telemetry or a debug system
        }
    }

    // ========== SERVO CONTROL ==========

    public void setTargetAngle(double angle) {
        targetAngle = clampServoAngle(angle);
    }
    public void aimAtTag(FieldPose robotPose, int tagId) {

        FieldPose tagPose = getAprilTagPosition(tagId);
        if (tagPose == null) {
            return; // Unknown tag
        }

        // get robot center position
        Pose2D robotCenter = coordinateTransformer.convertReferencePointToRobotCenter(
                robotPose.x, robotPose.y, robotPose.heading
        );

        FieldPose robotCenterPose = new FieldPose(
                robotCenter.getX(DistanceUnit.INCH),
                robotCenter.getY(DistanceUnit.INCH),
                robotCenter.getHeading(AngleUnit.DEGREES)
        );
        FieldPose cameraPos = calculateCameraPositionFromRobot(robotCenterPose);

        // Calculate distance from camera to tag (for aiming accuracy)
        double distanceToTag = calculateDistance(cameraPos, tagPose);

        // Calculate angle from CAMERA to tag (not from reference point!)
        double dx = tagPose.x - cameraPos.x;
        double dy = tagPose.y - cameraPos.y;
        double angleToTag = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to servo angle which is relative to robot heading, and need remove camera offset
        double servoAngle = normalizeAngle(angleToTag - robotPose.heading - CAMERA_OFFSET_HEADING);

        // Check if target is within servo usage range
        if (Math.abs(servoAngle) <= SERVO_USAGE_MAX_ANGLE) {
            setTargetAngle(servoAngle);

            // Distance-based behavior
            boolean withinReliableRange = (distanceToTag >= MIN_DETECTION_DISTANCE &&
                    distanceToTag <= MAX_DETECTION_DISTANCE);

            if (withinReliableRange) {
                // Within reliable range: stop searching, enable full detection
                stopSearch();
            } else {
                // Outside reliable range: aim at predicted position but don't stop searching
                // This allows the robot to point the camera correctly while continuing to search
                // for a better position or closer approach
            }
        }
    }

    public void startSearch(double centerAngle) {
        if (isSearching) return;

        isSearching = true;
        searchStep = 0;
        setTargetAngle(centerAngle - SEARCH_RANGE_DEGREES);
        searchTimer.reset();
    }

    /**
     * Updates search pattern if active
     *
     * @param centerAngle Center angle for search pattern
     * @return true if search is complete
     */
    public boolean updateSearch(double centerAngle) {
        if (!isSearching) return true;

        // Check if we've been at current position long enough
        if (searchTimer.milliseconds() < SEARCH_DWELL_TIME_MS) {
            return false;
        }

        // Move to next search position
        searchStep++;
        double stepAngle = centerAngle - SEARCH_RANGE_DEGREES + (searchStep * SEARCH_STEP_DEGREES);

        if (stepAngle > centerAngle + SEARCH_RANGE_DEGREES) {
            // Search complete - return to center
            stopSearch();
            setTargetAngle(centerAngle);
            return true;
        }

        setTargetAngle(stepAngle);
        searchTimer.reset();

        return false;
    }

    public void stopSearch() {
        isSearching = false;
        searchStep = 0;
    }


    private void updateServoPosition() {
        if (Math.abs(targetAngle - currentAngle) <= SERVO_POSITION_TOLERANCE) {
            return; // Already at target
        }

        // Calculate maximum angle change this update - by prediction
        double deltaTime = timer.seconds();
        timer.reset();
        double maxAngleChange = SERVO_MAX_SPEED * deltaTime;

        // Move towards target with velocity limiting
        double angleError = targetAngle - currentAngle;
        double angleChange = Math.signum(angleError) * Math.min(Math.abs(angleError), maxAngleChange);

        currentAngle += angleChange;
        currentAngle = clampServoAngle(currentAngle);

        // Set servo position
        double servoPosition = angleToServoPosition(currentAngle);
        panServo.setPosition(servoPosition);
    }

    // ========== UTILITY METHODS ==========

    /**
     * Centers the servo (0 degree position)
     */
    public void moveToCenter() {
        setTargetAngle(0.0);
        stopSearch();
    }

    /**
     * Moves servo to minimum usage angle (-180 degrees)
     */
    public void moveToMin() {
        setTargetAngle(SERVO_USAGE_MIN_ANGLE);
        stopSearch();
    }

    /**
     * Moves servo to maximum usage angle (+180 degrees)
     */
    public void moveToMax() {
        setTargetAngle(SERVO_USAGE_MAX_ANGLE);
        stopSearch();
    }

    /**
     * Checks if search pattern is active
     */
    public boolean isSearching() {
        return isSearching;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public long getTimeSinceLastDetection() {
        return System.currentTimeMillis() - lastDetectionTime;
    }

    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

    public double getLastDetectedDistance() {
        return lastDetectedDistance;
    }

    public double getLastDetectedBearing() {
        return lastDetectedBearing;
    }

    public double getLastDetectedYaw() {
        return lastDetectedYaw;
    }

    public double getLastDetectedElevation() {
        return lastDetectedElevation;
    }

    public double getLastFtcPoseX() {
        return lastFtcPoseX;
    }

    public double getLastFtcPoseY() {
        return lastFtcPoseY;
    }



    /**
     * Get camera position from last detection
     */
    public FieldPose getLastCameraPosition() {
        return lastCameraPosition;
    }

    /**
     * Get servo position from last detection
     */
    public FieldPose getLastServoPosition() {
        return lastServoPosition;
    }

    /**
     * Get current camera position based on robot pose and servo angle
     */
    public FieldPose getCurrentCameraPosition() {
        if (coordinateTransformer == null) {
            return new FieldPose(0, 0, 0);
        }
        Pose2D robotPose2D = coordinateTransformer.getCurrentRobotCenterPose();
        FieldPose robotPose = new FieldPose(
                robotPose2D.getX(DistanceUnit.INCH),
                robotPose2D.getY(DistanceUnit.INCH),
                robotPose2D.getHeading(AngleUnit.DEGREES)
        );
        return calculateCameraPositionFromRobot(robotPose);
    }

    /**
     * Get reference point position from raw odometry (no vision correction)
     * This is what the encoders/IMU think the robot position is
     */
    public FieldPose getOdometryReferencePoint() {
        if (odometryManager == null) {
            return new FieldPose(0, 0, 0);
        }
        Pose2D refPointPose2D = odometryManager.getCurrentPose();
        if (refPointPose2D != null) {
            return new FieldPose(
                    refPointPose2D.getX(DistanceUnit.INCH),
                    refPointPose2D.getY(DistanceUnit.INCH),
                    refPointPose2D.getHeading(AngleUnit.DEGREES)
            );
        }
        return new FieldPose(0, 0, 0);
    }

    /**
     * Get vision-based robot position from last AprilTag detection (pure vision, no fusion)
     * This is what the camera/AprilTag system calculates the robot position should be
     */
    public FieldPose getVisionBasedPosition() {
        if (lastDetection == null || getTimeSinceLastDetection() > 3000) {
            return null; // No recent detection available
        }
        return getVisionPoseFromDetection(lastDetection);
    }



    /**
     * Checks if last detection was within reliable distance range
     */
    public boolean isLastDetectionReliable() {
        return (lastDetectedDistance >= MIN_DETECTION_DISTANCE &&
                lastDetectedDistance <= MAX_DETECTION_DISTANCE);
    }

    /**
     * Gets minimum reliable detection distance
     */
    public double getMinDetectionDistance() {
        return MIN_DETECTION_DISTANCE;
    }

    /**
     * Gets maximum reliable detection distance
     */
    public double getMaxDetectionDistance() {
        return MAX_DETECTION_DISTANCE;
    }

    /**
     * Gets comprehensive status string
     */
    public String getStatus() {
        String reliabilityStatus = "";
        if (lastDetectedDistance > 0) {
            reliabilityStatus = isLastDetectionReliable() ? " RELIABLE" : " PREDICT";
        }

        String autoOdoStatus = autoOdometryCorrection ? " AUTO-ODO" : "";

        return String.format(Locale.US, "CameraServo: Angle=%.1f°->%.1f° | Target=%d | Detected=%d | Dist=%.1f\"%s | FW=%.0f RPM | %s%s%s",
                currentAngle, targetAngle, targetTagId, lastDetectedTagId, lastDetectedDistance,
                reliabilityStatus, lastFlywheelVelocity,
                isSearching ? " SEARCHING" : "", autoOdoStatus);
    }

    // ========== APRILTAG POSITION LOOKUP ==========

    /**
     * Gets AprilTag position from FieldPositions
     *
     * @param tagId AprilTag ID
     * @return FieldPose for the tag, or null if not found
     */
    private FieldPose getAprilTagPosition(int tagId) {
        switch (tagId) {
            case 20: // Blue GOAL
                return FieldPositions.GOAL_APRILTAG;
            case 24: // Red GOAL
                return FieldPositions.getRedPosition(FieldPositions.GOAL_APRILTAG);
            case 21: // OBELISK face 1
                return FieldPositions.OBELISK_TAG_21;
            case 22: // OBELISK face 2
                return FieldPositions.OBELISK_TAG_22;
            case 23: // OBELISK face 3
                return FieldPositions.OBELISK_TAG_23;
            default:
                return null; // Unknown tag
        }
    }

    // ========== COORDINATE SYSTEM METHODS ==========

    /**
     * Calculates camera position from robot position using camera offset and current servo angle
     * CRITICAL: Camera is mounted on a servo, so servo angle affects camera orientation
     *
     * @param robotPose Current robot CENTER pose (camera offsets are relative to robot center)
     * @return Camera pose in field coordinates including servo angle
     */
    private FieldPose calculateCameraPositionFromRobot(FieldPose robotPose) {
        // Apply camera offset rotated by robot heading
        double headingRad = Math.toRadians(robotPose.heading);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // Transform camera offset from robot frame to field frame
        double cameraX = robotPose.x + (CAMERA_OFFSET_X * cos - CAMERA_OFFSET_Y * sin);
        double cameraY = robotPose.y + (CAMERA_OFFSET_X * sin + CAMERA_OFFSET_Y * cos);

        // CRITICAL: Add servo angle to camera heading (camera is on servo)
        double cameraHeading = robotPose.heading + currentAngle;

        return new FieldPose(cameraX, cameraY, cameraHeading);
    }



    // ========== PRIVATE UTILITY METHODS ==========
    private static double angleToServoPosition(double angle) {
        //angle = angle + CAMERA_OFFSET_HEADING; // Remove camera offset

        angle = Math.max(SERVO_MIN_ANGLE, Math.min(SERVO_MAX_ANGLE, angle));

        double normalizedAngle = (angle + SERVO_MAX_ANGLE) / SERVO_RANGE_DEGREES;
        if (SERVO_DIRECTION_CCW) { // servo is CCW, and different with robot frame
            normalizedAngle = 1.0 - normalizedAngle;
        }
        return  Math.max(SERVO_MIN_POSITION, Math.min(SERVO_MAX_POSITION, normalizedAngle));
    }

    /**
     * Converts servo position to servo angle
     */
    private static double servoPositionToAngle(double position) {
        position = Math.max(SERVO_MIN_POSITION, Math.min(SERVO_MAX_POSITION, position));
        return (position * SERVO_RANGE_DEGREES) - SERVO_MAX_ANGLE;
    }

    /**
     * Clamps servo angle to practical usage range (±180°)
     * Hardware supports ±810° but we limit usage to ±180° for camera coverage
     */
    private static double clampServoAngle(double angle) {
        return Math.max(SERVO_USAGE_MIN_ANGLE, Math.min(SERVO_USAGE_MAX_ANGLE, angle));
    }

    /**
     * Normalizes angle to [-180, +180] range
     */
    private static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    /**
     * Cleanup method to properly close vision system resources
     * Should be called when the OpMode ends
     */
    public void cleanup() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTagProcessor = null;
    }


}




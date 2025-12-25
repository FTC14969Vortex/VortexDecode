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
    
    /** Total servo range in degrees (GoBilda 5-turn servo: 4.5 turns = 1620°) */
    private static final double SERVO_RANGE_DEGREES = 1620.0;
    
    /** Maximum servo angle from center (+810 degrees) - HARDWARE LIMIT */
    private static final double SERVO_MAX_ANGLE = 810.0;
    
    /** Minimum servo angle from center (-810 degrees) - HARDWARE LIMIT */
    private static final double SERVO_MIN_ANGLE = -810.0;
    
    /** Maximum USAGE angle from center (+180 degrees) - PRACTICAL LIMIT */
    private static final double SERVO_USAGE_MAX_ANGLE = 180.0;
    
    /** Minimum USAGE angle from center (-180 degrees) - PRACTICAL LIMIT */
    private static final double SERVO_USAGE_MIN_ANGLE = -180.0;
    
    /** Servo center position (0.5 = center) */
    private static final double SERVO_CENTER_POSITION = 0.5;
    
    /** Minimum servo position (0.0) */
    private static final double SERVO_MIN_POSITION = 0.0;
    
    /** Maximum servo position (1.0) */
    private static final double SERVO_MAX_POSITION = 1.0;
    
    /** Maximum servo speed in degrees per second */
    private static final double SERVO_MAX_SPEED = 600.0;  // GoBilda spec
    
    /** Servo position tolerance for "at target" detection */
    private static final double SERVO_POSITION_TOLERANCE = 2.0;  // degrees
    
    /** 
     * Servo direction multiplier - controls camera rotation direction
     * +1.0 = Standard mounting: positive angle → camera rotates LEFT (counterclockwise)
     * -1.0 = Reversed mounting: positive angle → camera rotates RIGHT (clockwise)
     * 
     * COORDINATE SYSTEM CONVENTION:
     * When looking along camera's facing direction (from behind camera):
     * - Positive servo angle should rotate camera LEFT (counterclockwise)
     * - Negative servo angle should rotate camera RIGHT (clockwise)
     * 
     * If physical servo behavior is opposite, set this to -1.0
     */
    private static final double SERVO_DIRECTION_MULTIPLIER = 1.0;
    
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
    
    /** Camera offset from robot center (inches, degrees) - Using BACK_CAMERA from RobotConstants */
    private static final double CAMERA_OFFSET_X = RobotConstants.BACK_CAMERA.x;    // Forward/backward from robot center
    private static final double CAMERA_OFFSET_Y = RobotConstants.BACK_CAMERA.y;    // Left/right from robot center  
    private static final double CAMERA_OFFSET_HEADING = RobotConstants.BACK_CAMERA.yaw; // Rotation offset from robot heading
    
    // ========== FLYWHEEL VELOCITY PARAMETERS ==========
    
    /** Linear flywheel velocity dependence on distance */
    private static final double FLYWHEEL_VELOCITY_SLOPE = 8.0;      // RPM per inch
    private static final double FLYWHEEL_VELOCITY_INTERCEPT = 800.0; // Base RPM
    
    /** Minimum flywheel velocity (RPM) */
    private static final double MIN_FLYWHEEL_VELOCITY = 800.0;
    
    /** Maximum flywheel velocity (RPM) */
    private static final double MAX_FLYWHEEL_VELOCITY = 1500.0;
    
    // ========== POSE FUSION PARAMETERS ==========
    
    /** Vision weight for pose fusion (0.0 = odometry only, 1.0 = vision only) */
    private static final double VISION_FUSION_WEIGHT = 0.3;
    
    /** Maximum distance for pose correction (inches) */
    private static final double MAX_CORRECTION_DISTANCE = 60.0;
    
    // ========== DETECTION DISTANCE LIMITS ==========
    
    /** Minimum reliable detection distance (inches) */
    private static final double MIN_DETECTION_DISTANCE = 40.0;
    
    /** Maximum reliable detection distance (inches) */
    private static final double MAX_DETECTION_DISTANCE = 120.0;
    
    // ========== AUTOMATIC ODOMETRY CORRECTION ==========
    
    /** Enable automatic odometry correction when AprilTags are detected */
    private boolean autoOdometryCorrection = true;
    
    // ========== HARDWARE ==========
    
    private Servo panServo;
    private AprilTagProcessor aprilTagProcessor;
    private OdometryManager odometryManager;
    private CoordinateTransformer coordinateTransformer;
    private MotionExecutor motionExecutor;
    private ElapsedTime timer;
    private ElapsedTime searchTimer;
    
    // ========== STATE TRACKING ==========
    
    private double currentAngle = 0.0;      // Current servo angle in degrees (0 = forward)
    private double targetAngle = 0.0;       // Target servo angle in degrees
    private boolean isInitialized = false;
    private boolean isSearching = false;
    private int searchStep = 0;
    
    // Vision state
    private double lastFlywheelVelocity = MIN_FLYWHEEL_VELOCITY;
    private long lastDetectionTime = 0;
    private int lastDetectedTagId = -1;
    private double lastDetectedDistance = 0.0;

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
     * @param aprilTagProcessor AprilTag processor from vision portal
     * @param odometryManager Odometry manager for automatic pose corrections (optional)
     * @param coordinateTransformer Coordinate transformer for pose calculations (optional)
     * @param motionExecutor Motion executor for robot rotation during shooting alignment (optional)
     */
    public void init(HardwareMap hardwareMap, AprilTagProcessor aprilTagProcessor, OdometryManager odometryManager, CoordinateTransformer coordinateTransformer, MotionExecutor motionExecutor) {
        try {
            // Initialize servo hardware
            panServo = hardwareMap.get(Servo.class, CAMERA_SERVO_NAME);
            this.aprilTagProcessor = aprilTagProcessor;
            this.odometryManager = odometryManager;
            this.coordinateTransformer = coordinateTransformer;
            this.motionExecutor = motionExecutor;
            timer = new ElapsedTime();
            searchTimer = new ElapsedTime();
            
            // Set to center position on init
            setTargetAngle(0.0);
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
     * @param aprilTagProcessor AprilTag processor from vision portal
     */
    public void init(HardwareMap hardwareMap, AprilTagProcessor aprilTagProcessor) {
        init(hardwareMap, aprilTagProcessor, null, null, null);
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
        
        // Get current robot pose for prediction
        if (coordinateTransformer != null) {
            Pose2D robotCenterPose2D = coordinateTransformer.getCurrentRobotCenterPose();
            // Convert robot center to reference point for aimAtTag (which expects reference point)
            FieldPose referencePointPose = coordinateTransformer.convertRobotCenterToReferencePoint(robotCenterPose2D);
            
            // Always aim camera at predicted target tag position
            aimAtTag(referencePointPose, targetTagId);
        }
        
        // Update search pattern if active
        if (isSearching) {
            updateSearch(targetAngle);
        }
        
        // Update servo position with velocity limiting
        updateServoPosition();
        
        // Process AprilTag detections and update flywheel velocity
        processAprilTagDetections();
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
                Thread.sleep(10); // 10ms sleep for responsive shutdown
                
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
        
        // Get current robot pose to determine distance to target
        FieldPose robotPose = null;
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
                FieldPose cameraPos = calculateCameraPositionFromRobot(robotCenterPose);
                predictedDistance = calculateDistance(cameraPos, tagPose);
                withinReliableRange = (predictedDistance >= MIN_DETECTION_DISTANCE && 
                                     predictedDistance <= MAX_DETECTION_DISTANCE);
            }
        }
        
        if (targetDetection != null) {
            // Tag detected - update state and stop searching
            lastDetectionTime = System.currentTimeMillis();
            lastDetectedTagId = targetDetection.id;
            lastDetectedDistance = calculateDistance(targetDetection);
            
            // Calculate and update flywheel velocity
            lastFlywheelVelocity = calculateFlywheelVelocity(lastDetectedDistance);
            
            // Perform automatic odometry correction if within reliable range
            if (withinReliableRange) {
                performAutomaticOdometryCorrection(targetDetection);
            }
            
            // Stop searching since we found the target
            if (isSearching) {
                stopSearch();
            }
        } else {
            // Tag not detected - start search if within reliable range
            if (withinReliableRange && !isSearching) {
                // Start search pattern around current predicted angle
                startSearch(targetAngle);
            }
            // If outside reliable range, don't search (just aim at predicted position)
            // Keep last flywheel velocity when no detection
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
     * 
     * NOTE: detection.range should equal Math.sqrt(dx*dx + dy*dy) from detection.ftcPose
     * when both are available. detection.range is the direct distance measurement from SDK,
     * while ftcPose calculation gives the same result from X,Y components.
     * 
     * Uses detection.range when available (direct tag-to-camera distance),
     * otherwise calculates using tag position and camera position
     */
    private double calculateDistance(AprilTagDetection detection) {
        // PREFERRED: Use detection.ftcPose.range if available (direct tag-to-camera distance from SDK)
        if (detection.ftcPose != null && detection.ftcPose.range > 0) {
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
            FieldPose cameraPose = calculateCameraPositionFromRobot(robotPose);
            return calculateDistance(tagPose, cameraPose);
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
        
        // Validate result is not NaN or negative
        if (distanceSquared < 0 || Double.isNaN(distanceSquared)) {
            return 0.0;
        }
        
        return Math.sqrt(distanceSquared);
    }
    

    
    // ========== FLYWHEEL VELOCITY CALCULATION ==========
    
    /**
     * Gets current flywheel velocity based on detected AprilTag distance
     * 
     * @return Flywheel velocity in RPM
     */
    public double getFlywheelVelocity() {
        return lastFlywheelVelocity;
    }
    
    /**
     * Calculates flywheel velocity for given distance using linear model
     * Formula: velocity = SLOPE * distance + INTERCEPT
     * 
     * @param distance Distance to target in inches
     * @return Flywheel velocity in RPM (clamped to min/max)
     */
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
    public FieldPose getCorrectedPose(FieldPose odometryPose) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        
        if (detections == null || detections.isEmpty()) {
            return odometryPose; // No vision data - return odometry
        }
        
        // Find target tag for pose correction
        AprilTagDetection targetDetection = findTargetTag(detections);
        
        if (targetDetection == null || getAprilTagPosition(targetDetection.id) == null) {
            return odometryPose; // Target tag not found
        }
        
        // Get vision-based robot pose from detection (this is what detection.robotPose represents!)
        FieldPose visionPose = getVisionPoseFromDetection(targetDetection);
        
        if (visionPose == null) {
            return odometryPose; // Vision pose not available
        }
        
        // Check if correction distance is reasonable (ignore on first detection)
        double correctionDistance = calculateDistance(odometryPose, visionPose);
        if (!hasInitializedPosition) {
            // First detection - accept any distance to establish initial position
            hasInitializedPosition = true;
        } else if (correctionDistance > MAX_CORRECTION_DISTANCE) {
            return odometryPose; // Subsequent detections - reject if correction too large
        }
        
        // Fuse odometry and vision poses
        return fusePoses(odometryPose, visionPose, VISION_FUSION_WEIGHT);
    }
    
    /**
     * Calculates robot pose from camera detection using mathematically correct transformation
     * CRITICAL: Camera is on servo, so servo angle affects the transformation
     * 
     * COORDINATE SYSTEM UNDERSTANDING:
     * - detection.ftcPose.x,y,yaw = Tag position/orientation relative to camera (in camera frame)
     * - We need to find robot center position in field frame
     * 
     * TRANSFORMATION CHAIN:
     * 1. Invert detection: Camera position relative to tag (tag frame)
     * 2. Transform camera to robot center (account for servo, mounting, offset)
     * 3. Transform robot center to reference point (unified coordinate system)
     * 4. Transform reference point position from tag frame to field frame
     */
    private FieldPose getVisionPoseFromDetection(AprilTagDetection detection) {
        if (detection.ftcPose == null) {
            return null;
        }
        
        // Get known tag position in field coordinates
        FieldPose knownTagPose = getAprilTagPosition(detection.id);
        if (knownTagPose == null) {
            return null;
        }
        
        // STEP 1: Invert detection to get camera position relative to tag
        // detection.ftcPose gives tag position in camera frame
        // We need camera position in tag frame (inverse transformation)
        double tagFrameCameraX = -detection.ftcPose.x;
        double tagFrameCameraY = -detection.ftcPose.y;
        double tagFrameCameraYaw = -detection.ftcPose.yaw;
        
        // STEP 2: Transform camera position to robot center position (in tag frame)
        
        // 2a: Account for servo rotation (camera is rotated by servo angle)
        double servoAngleRad = Math.toRadians(currentAngle * SERVO_DIRECTION_MULTIPLIER);
        double cosServo = Math.cos(servoAngleRad);
        double sinServo = Math.sin(servoAngleRad);
        
        // Rotate camera position by servo angle to get position in servo-mount frame
        double servoMountCameraX = tagFrameCameraX * cosServo - tagFrameCameraY * sinServo;
        double servoMountCameraY = tagFrameCameraX * sinServo + tagFrameCameraY * cosServo;
        
        // 2b: Account for camera mounting orientation (180° for back-facing camera)
        // Back camera faces backward, so 180° rotation from robot forward direction
        double robotFrameCameraX = -servoMountCameraX;  // 180° rotation: x' = -x
        double robotFrameCameraY = -servoMountCameraY;  // 180° rotation: y' = -y
        double robotFrameCameraYaw = normalizeAngle(tagFrameCameraYaw + (currentAngle * SERVO_DIRECTION_MULTIPLIER) + CAMERA_OFFSET_HEADING);
        
        // 2c: Account for camera offset from robot center
        // Camera is mounted at offset position from robot center
        double robotCenterX = robotFrameCameraX - CAMERA_OFFSET_X;
        double robotCenterY = robotFrameCameraY - CAMERA_OFFSET_Y;
        
        // 2d: Convert robot center to reference point position (in tag frame)
        // With new architecture, we need to return reference point coordinates
        double refPointX = robotCenterX + MotionConfig.ACTIVE_REFERENCE_POINT.x;
        double refPointY = robotCenterY + MotionConfig.ACTIVE_REFERENCE_POINT.y;
        
        // STEP 3: Transform reference point position from tag frame to field frame
        // Now refPointX/Y represent reference point position in tag-relative coordinates
        // Transform to field coordinates using tag's known field position and orientation
        double tagHeadingRad = Math.toRadians(knownTagPose.heading);
        double cosTag = Math.cos(tagHeadingRad);
        double sinTag = Math.sin(tagHeadingRad);
        
        // Apply 2D rotation matrix to transform from tag frame to field frame
        double fieldX = knownTagPose.x + (refPointX * cosTag - refPointY * sinTag);
        double fieldY = knownTagPose.y + (refPointX * sinTag + refPointY * cosTag);
        double fieldHeading = normalizeAngle(knownTagPose.heading + robotFrameCameraYaw);
        
        return new FieldPose(fieldX, fieldY, fieldHeading);
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
        
        // CRITICAL FIX: Proper circular averaging for angles
        // Convert angles to unit vectors, average the vectors, then convert back to angle
        double heading1Rad = Math.toRadians(pose1.heading);
        double heading2Rad = Math.toRadians(pose2.heading);
        
        // Convert to unit vectors
        double x1 = Math.cos(heading1Rad);
        double y1 = Math.sin(heading1Rad);
        double x2 = Math.cos(heading2Rad);
        double y2 = Math.sin(heading2Rad);
        
        // Weighted average of unit vectors
        double avgX = x1 * weight1 + x2 * weight2;
        double avgY = y1 * weight1 + y2 * weight2;
        
        // Convert averaged vector back to angle
        double fusedHeadingRad = Math.atan2(avgY, avgX);
        double fusedHeading = Math.toDegrees(fusedHeadingRad);
        
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
        
        // Get current odometry pose (now returns reference point coordinates directly)
        FieldPose currentOdometryPose = null;
        if (odometryManager != null) {
            Pose2D refPointPose2D = odometryManager.getCurrentPose();
            if (refPointPose2D != null) {
                currentOdometryPose = new FieldPose(refPointPose2D.getX(DistanceUnit.INCH), 
                                                   refPointPose2D.getY(DistanceUnit.INCH), 
                                                   refPointPose2D.getHeading(AngleUnit.DEGREES));
            }
        }
        
        if (currentOdometryPose == null) {
            return; // No current odometry pose available
        }
        
        // Get vision-based reference point pose from detection
        FieldPose visionPose = getVisionPoseFromDetection(detection);
        if (visionPose == null) {
            return; // Vision pose not available
        }
        
        // Check if correction distance is reasonable (ignore on first detection)
        double correctionDistance = calculateDistance(currentOdometryPose, visionPose);
        if (!hasInitializedPosition) {
            // First detection - accept any distance to establish initial position
            hasInitializedPosition = true;
        } else if (correctionDistance > MAX_CORRECTION_DISTANCE) {
            return; // Subsequent detections - reject if correction too large
        }
        
        // Fuse odometry and vision poses (both now in reference point coordinates)
        // This eliminates coordinate frame mismatch and provides mathematically correct fusion
        FieldPose fusedPose = fusePoses(currentOdometryPose, visionPose, VISION_FUSION_WEIGHT);
        
        // Apply the fused correction to odometry
        try {
            // Convert FieldPose to Pose2D for odometry manager
            Pose2D correctionPose = new Pose2D(
                DistanceUnit.INCH, fusedPose.x, fusedPose.y,
                AngleUnit.DEGREES, fusedPose.heading
            );
            
            // Apply the fused correction to odometry
            odometryManager.resetToPose(correctionPose);
            
        } catch (Exception e) {
            // Silently handle errors to avoid disrupting robot operation
            // In production, this could log to telemetry or a debug system
        }
    }
    

    
    // ========== SERVO CONTROL ==========
    
    public void setTargetAngle(double angle) {
        targetAngle = clampServoAngle(angle);
    }
    
    /**
     * Aims servo at specific AprilTag based on robot position
     * Implements distance-based behavior: prediction mode when outside reliable range
     * 
     * @param robotPose Current robot reference point pose (from odometry)
     * @param tagId Target AprilTag ID
     */
    public void aimAtTag(FieldPose robotPose, int tagId) {
        if (coordinateTransformer == null) {
            return; // Cannot aim without coordinate transformer
        }
        
        FieldPose tagPose = getAprilTagPosition(tagId);
        if (tagPose == null) {
            return; // Unknown tag
        }
        
        // CRITICAL: Convert reference point to robot center, then to camera position
        // robotPose is reference point from odometry, but calculateCameraPositionFromRobot expects robot center
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
        
        // Convert to servo angle (relative to robot heading, not camera heading)
        double servoAngle = normalizeAngle(angleToTag - robotPose.heading) * SERVO_DIRECTION_MULTIPLIER;
        
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
    
    // ========== SHOOTING ALIGNMENT ==========
    
    /**
     * Complete shooting alignment method - handles everything internally
     * 
     * This method:
     * 1. Resets camera to center position (servo at 0°)
     * 2. Calculates required robot rotation using odometry + known tag positions
     * 3. Executes robot rotation to align camera with tag
     * 4. Updates lastDetectedDistance and lastFlywheelVelocity for retrieval
     * 
     * Works with or without tag detection:
     * - If tag detected: Uses actual detection for accurate measurements
     * - If no tag: Uses odometry calculations for all parameters
     * 
     * @param targetTagId AprilTag ID to align with
     * @return true if alignment successful, false if prerequisites missing
     */
    public boolean alignForShooting(int targetTagId) {
        // Validate prerequisites
        if (coordinateTransformer == null) {
            return false; // Need coordinate transformer for robot pose
        }
        
        FieldPose tagPose = getAprilTagPosition(targetTagId);
        if (tagPose == null) {
            return false; // Unknown tag ID
        }
        
        // Step 1: Reset camera to center position and wait for movement
        center(); // Use existing method
        
        // Calculate predicted time to reach center based on current position and servo speed
        double angleToMove = Math.abs(currentAngle - 0.0);
        double predictedTimeMs = (angleToMove / SERVO_MAX_SPEED) * 1000.0;
        double waitTimeMs = Math.min(predictedTimeMs, 200.0); // Max 200ms as requested
        
        try {
            Thread.sleep((long)waitTimeMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // Step 2: Get current robot center pose and calculate camera position
        Pose2D robotCenterPose2D = coordinateTransformer.getCurrentRobotCenterPose();
        FieldPose robotCenterPose = new FieldPose(
            robotCenterPose2D.getX(DistanceUnit.INCH),
            robotCenterPose2D.getY(DistanceUnit.INCH),
            robotCenterPose2D.getHeading(AngleUnit.DEGREES)
        );
        
        // Calculate camera position for accurate shooting alignment
        FieldPose cameraPos = calculateCameraPositionFromRobot(robotCenterPose);
        
        // Step 3: Calculate required robot rotation from CAMERA to tag (not robot center to tag)
        double dx = tagPose.x - cameraPos.x;
        double dy = tagPose.y - cameraPos.y;
        double angleToTag = Math.toDegrees(Math.atan2(dy, dx));
        double rotationNeeded = normalizeAngle(angleToTag - robotCenterPose.heading);
        
        // Step 4: Execute robot rotation if significant misalignment
        if (motionExecutor != null && Math.abs(rotationNeeded) > 1.0) {
            try {
                motionExecutor.rotate(rotationNeeded);
                
                // Wait for rotation to complete
                while (motionExecutor.isExecuting()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
            } catch (Exception e) {
                // If rotation fails, continue with current position
            }
        }
        
        // Step 5: Get final measurements and update class variables
        // Try to get actual detection first for accuracy
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection detection = findTargetTag(detections);
        
        if (detection != null) {
            // Use actual detection for accurate measurements
            lastDetectedDistance = calculateDistance(detection);
            lastDetectedTagId = targetTagId;
            lastDetectionTime = System.currentTimeMillis();
        } else {
            // Fall back to odometry calculation using camera position
            Pose2D fallbackRobotCenterPose2D = coordinateTransformer.getCurrentRobotCenterPose();
            FieldPose fallbackRobotCenterPose = new FieldPose(
                fallbackRobotCenterPose2D.getX(DistanceUnit.INCH),
                fallbackRobotCenterPose2D.getY(DistanceUnit.INCH),
                fallbackRobotCenterPose2D.getHeading(AngleUnit.DEGREES)
            );
            // Calculate distance from camera to tag (not robot center to tag)
            FieldPose fallbackCameraPos = calculateCameraPositionFromRobot(fallbackRobotCenterPose);
            lastDetectedDistance = calculateDistance(fallbackCameraPos, tagPose);
            lastDetectedTagId = targetTagId;
            lastDetectionTime = System.currentTimeMillis();
        }
        
        // Calculate and store flywheel velocity
        lastFlywheelVelocity = calculateFlywheelVelocity(lastDetectedDistance);
        
        return true;
    }
    
    private void updateServoPosition() {
        if (Math.abs(targetAngle - currentAngle) <= SERVO_POSITION_TOLERANCE) {
            return; // Already at target
        }
        
        // Calculate maximum angle change this update
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
    public void center() {
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
     * Checks if servo is moving to target
     */
    public boolean isMoving() {
        return Math.abs(targetAngle - currentAngle) > SERVO_POSITION_TOLERANCE;
    }
    
    /**
     * Checks if search pattern is active
     */
    public boolean isSearching() {
        return isSearching;
    }
    
    /**
     * Gets current servo angle
     * 
     * IMPORTANT: The servo angle directly represents robot misalignment from perfect shooting alignment.
     * - Perfect alignment: servo angle = 0° (camera at center position facing tag center)
     * - Positive angle: robot needs to turn RIGHT to align
     * - Negative angle: robot needs to turn LEFT to align
     * 
     * @return Current servo angle in degrees (robot misalignment angle)
     */
    public double getCurrentAngle() {
        return currentAngle;
    }
    
    /**
     * Gets target servo angle
     */
    public double getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * Gets time since last AprilTag detection
     */
    public long getTimeSinceLastDetection() {
        return System.currentTimeMillis() - lastDetectionTime;
    }
    
    /**
     * Gets last detected AprilTag ID
     */
    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }
    
    /**
     * Gets last detected distance
     */
    public double getLastDetectedDistance() {
        return lastDetectedDistance;
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
                           reliabilityStatus, lastFlywheelVelocity, isMoving() ? "MOVING" : "STOPPED",
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
                return FieldPositions.BLUE_GOAL_APRILTAG;
            case 24: // Red GOAL  
                return FieldPositions.RED_GOAL_APRILTAG;
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
        double cameraHeading = robotPose.heading + CAMERA_OFFSET_HEADING + (currentAngle * SERVO_DIRECTION_MULTIPLIER);
        
        return new FieldPose(cameraX, cameraY, cameraHeading);
    }
    

    
    // ========== PRIVATE UTILITY METHODS ==========
    
    /**
     * Converts servo angle to servo position
     */
    private static double angleToServoPosition(double angle) {
        angle = Math.max(SERVO_MIN_ANGLE, Math.min(SERVO_MAX_ANGLE, angle));
        double normalizedAngle = (angle + SERVO_MAX_ANGLE) / SERVO_RANGE_DEGREES;
        return Math.max(SERVO_MIN_POSITION, Math.min(SERVO_MAX_POSITION, normalizedAngle));
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
}

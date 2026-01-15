package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.calibration.CalibrationUtils.StatusType;
import org.firstinspires.ftc.teamcode.calibration.modules.OdometryCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.OdometryDirectionCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.MotorDirectionCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.MotorVelocityPIDFCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.AxisPIDCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.HeadingPIDCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.HybridControlCalibrationModule;
import org.firstinspires.ftc.teamcode.calibration.modules.KinematicMatrixCalibrationModule;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.OdometryManager;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;

/**
 * Motion Calibration and Demo System - Modular Architecture
 * 
 * This OpMode serves as a lightweight controller that manages multiple
 * calibration modules through FTC Dashboard. Each calibration is implemented
 * as a separate module for maintainability and extensibility.
 * 
 * FEATURES:
 * - Single OpMode interface for all calibrations
 * - Modular architecture - easy to add new calibrations
 * - Real-time parameter tuning via FTC Dashboard
 * - Dropdown menu mode selection (no more typing numbers!)
 * - Each module handles its own @Config parameters
 * - Dynamic interface based on selected calibration mode
 * 
 * CALIBRATION MODULES:
 * 0. OdometryCalibrationModule - Odometry distance corrections
 * 1. OdometryDirectionCalibrationModule - Odometry encoder directions
 * 2. MotorDirectionCalibrationModule - Motor and encoder directions
 * 3. MotorVelocityPIDFCalibrationModule - Motor velocity control
 * 4. AxisPIDCalibrationModule - X/Y axis position control (consolidated)
 * 5. HeadingPIDCalibrationModule - Rotation control
 * 6. HybridControlCalibrationModule - Advanced optimization
 * 7. KinematicMatrixCalibrationModule - Kinematic matrix calibration
 * 
 * USAGE WORKFLOW:
 * 1. Connect to FTC Dashboard at http://192.168.43.1:8080/dash
 * 2. Select calibration mode using the dropdown menu in _0_ModeSelector
 * 3. Adjust parameters in real-time for the selected mode
 * 4. Toggle ENABLE_TESTING to run calibration tests
 * 5. Observe performance metrics and recommendations
 * 6. WRITE DOWN the optimized values from Dashboard
 * 7. Manually update your source code files with the calibrated values
 * 8. Refer to your comprehensive calibration manual for guidance
 * 
 * IMPORTANT: FTC Dashboard cannot save directly to source files.
 * You must manually copy calibrated values from Dashboard to your config files.
 */

@Disabled
@TeleOp(name = "🎯 Motion Calibration and Demo", group = "Calibration")
public class MotionCalibrationAndDemo extends LinearOpMode {
    
    // ========== CALIBRATION MODES ==========
    
    public enum CalibrationMode {
        ODOMETRY(0, "Odometry Calibration"),
        ODOMETRY_DIRECTION(1, "Odometry Direction"),
        MOTOR_DIRECTION(2, "Motor Direction"),
        MOTOR_VELOCITY_PIDF(3, "Motor Velocity PIDF"),
        DISTANCE_PID(4, "DISTANCE PID "),
        HEADING_PID(5, "Heading PID"),
        HYBRID_CONTROL(6, "Hybrid Control"),
        KINEMATIC_MATRIX(7, "Kinematic Matrix Calibration");
        
        public final int id;
        public final String name;
        
        CalibrationMode(int id, String name) {
            this.id = id;
            this.name = name;
        }
        
        public static CalibrationMode fromId(int id) {
            for (CalibrationMode mode : values()) {
                if (mode.id == id) return mode;
            }
            return ODOMETRY; // Default
        }
    }
    
    // ========== SELECTION ENUMS FOR DASHBOARD DROPDOWNS ==========
    
    /**
     * Motor selection enum for FTC Dashboard dropdown menu
     * Replaces integer-based motor selection for better UX
     */
    public enum MotorSelection {
        FRONT_LEFT(0, "Front Left"),
        FRONT_RIGHT(1, "Front Right"),
        BACK_LEFT(2, "Back Left"),
        BACK_RIGHT(3, "Back Right");
        
        public final int id;
        public final String displayName;
        
        MotorSelection(int id, String displayName) {
            this.id = id;
            this.displayName = displayName;
        }
    }
    
    /**
     * Axis selection enum for FTC Dashboard dropdown menu
     * Used in Axis PID calibration module
     */
    public enum AxisSelection {
        X_AXIS(0, "X Axis"),
        Y_AXIS(1, "Y Axis"),
        BOTH_AXES(2, "Both X and Y");
        
        public final int id;
        public final String displayName;
        
        AxisSelection(int id, String displayName) {
            this.id = id;
            this.displayName = displayName;
        }
    }
    
    /**
     * Velocity test selection enum for FTC Dashboard dropdown menu
     * Used in Motor Velocity PIDF calibration module
     */
    public enum VelocityTestSelection {
        TEST_FORWARD(0, "Forward (+X)"),
        TEST_BACKWARD(1, "Backward (-X)"),
        TEST_STRAFE_LEFT(2, "Strafe Left (+Y)"),
        TEST_STRAFE_RIGHT(3, "Strafe Right (-Y)"),
        RUN_ALL_TESTS(4, "Run All 4 Tests");
        
        public final int id;
        public final String displayName;
        
        VelocityTestSelection(int id, String displayName) {
            this.id = id;
            this.displayName = displayName;
        }
    }
    
    /**
     * Hybrid control test selection enum for FTC Dashboard dropdown menu
     * Used in Hybrid Control calibration module
     */
    public enum HybridTestSelection {
        TEST_0_000DEG(0, "Test 0: 000°", 0.0),
        TEST_1_045DEG(1, "Test 1: 045°", 45.0),
        TEST_2_090DEG(2, "Test 2: 090°", 90.0),
        TEST_3_135DEG(3, "Test 3: 135°", 135.0),
        TEST_4_180DEG(4, "Test 4: 180°", 180.0),
        TEST_5_225DEG(5, "Test 5: 225°", 225.0),
        TEST_6_270DEG(6, "Test 6: 270°", 270.0),
        TEST_7_315DEG(7, "Test 7: 315°", 315.0),
        RUN_ALL_TESTS(8, "Run All Tests", -1.0);
        
        public final int testIndex;
        public final String displayName;
        public final double angle;
        
        HybridTestSelection(int testIndex, String displayName, double angle) {
            this.testIndex = testIndex;
            this.displayName = displayName;
            this.angle = angle;
        }
    }
    
    /**
     * Calibration mode selection enum for FTC Dashboard dropdown menu
     * Replaces integer-based mode selection for better UX
     */
    public enum CalibrationModeEnum {
        ODOMETRY("Odometry Calibration", 0),
        ODOMETRY_DIRECTION("Odometry Direction", 1),
        MOTOR_DIRECTION("Motor Direction", 2),
        MOTOR_VELOCITY_PIDF("Motor Velocity PIDF", 3),
        DISTANCE_PID("DISTANCE_PID", 4),
        HEADING_PID("Heading PID", 5),
        HYBRID_CONTROL("Hybrid Control", 6),
        KINEMATIC_MATRIX("Kinematic Matrix", 7);
        
        public final String displayName;
        public final int id;
        
        CalibrationModeEnum(String displayName, int id) {
            this.displayName = displayName;
            this.id = id;
        }
    }
    
    // ========== DASHBOARD MODE SELECTOR ==========
    
    @Config
    public static class _0_ModeSelector {
        // Use dropdown menu to select calibration mode (much better than typing numbers!)
        public static CalibrationModeEnum CALIBRATION_MODE = CalibrationModeEnum.ODOMETRY;
        
        // Toggle to start/stop calibration testing
        public static boolean ENABLE_TESTING = false;
        
        // Toggle to reset current calibration parameters to default values
        public static boolean RESET_TO_DEFAULTS = false;
    }
    
    // ========== HARDWARE ==========
    
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private MotionExecutor motionExecutor;
    private OdometryManager odometryManager;
    
    // ========== CALIBRATION MODULES ==========
    
    private BaseCalibration[] calibrationModules;
    private BaseCalibration currentCalibration;
    
    // ========== STATE MANAGEMENT ==========
    
    private CalibrationMode currentMode = CalibrationMode.ODOMETRY;
    private CalibrationMode lastMode = CalibrationMode.ODOMETRY;
    private ElapsedTime runtime = new ElapsedTime();
    
    // FTC Dashboard integration
    private FtcDashboard dashboard;
    private SmartDashboardManager dashboardManager;
    
    @Override
    public void runOpMode() {
        
        // Initialize hardware
        initializeHardware();
        
        // Initialize FTC Dashboard with Smart Manager
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboardManager = new SmartDashboardManager();
        
        // Initialize calibration modules
        initializeCalibrationModules();
        
        // Display welcome screen
        displayWelcomeScreen();
        
        waitForStart();
        runtime.reset();
        
        // Main calibration loop
        while (opModeIsActive()) {
            
            // TIER 1: Fast internal updates (always 10Hz for calibration accuracy)
            updateCurrentMode();
            handleModeChange();
            handleParameterReset();
            handleTestControl();
            updateCurrentCalibration();
            
            // TIER 2: Configurable text telemetry (prevents dashboard refresh)
            if (dashboardManager.shouldUpdateTextTelemetry()) {
                displayStatus();
                telemetry.update();
            }
            
            // TIER 3: Graph data handled by individual calibration modules
            // (Each module calls dashboardManager.updateCalibrationData())
            
            sleep(100); // Keep 10Hz base loop for calibration accuracy
        }
        
        // Stop all systems
        stopAllSystems();
    }
    
    /**
     * Initialize hardware
     */
    private void initializeHardware() {
        try {
            // Initialize motors
            frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
            frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
            backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
            backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
            
            // Set motor modes
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // Set zero power behavior
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Initialize motion system (for position/heading calibrations)
            try {
                GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
                motionExecutor = new MotionExecutor(frontLeftDrive, frontRightDrive, 
                                                  backLeftDrive, backRightDrive, pinpoint);
                odometryManager = motionExecutor.getMotionState().getOdometryManager();
            } catch (Exception e) {
                telemetry.addLine("⚠️ Motion system not available - position calibrations limited");
            }
            
        } catch (Exception e) {
            telemetry.addLine(CalibrationUtils.formatStatus(
                "Hardware initialization failed: " + e.getMessage(), StatusType.ERROR));
            telemetry.update();
        }
    }
    
    /**
     * Initialize calibration modules
     */
    private void initializeCalibrationModules() {
        calibrationModules = new BaseCalibration[8];
        
        // Initialize all available modules
        calibrationModules[0] = new OdometryCalibrationModule();
        calibrationModules[1] = new OdometryDirectionCalibrationModule();
        calibrationModules[2] = new MotorDirectionCalibrationModule();
        calibrationModules[3] = new MotorVelocityPIDFCalibrationModule();
        calibrationModules[4] = new AxisPIDCalibrationModule();
        calibrationModules[5] = new HeadingPIDCalibrationModule();
        calibrationModules[6] = new HybridControlCalibrationModule();
        calibrationModules[7] = new KinematicMatrixCalibrationModule();
        
        // Initialize all modules with hardware
        for (int i = 0; i < calibrationModules.length; i++) {
            if (calibrationModules[i] != null) {
                calibrationModules[i].initialize(
                    hardwareMap, telemetry,
                    frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive,
                    motionExecutor, odometryManager, this
                );
            }
        }
        
        // Set initial calibration
        currentCalibration = calibrationModules[0];
    }
    
    /**
     * Display welcome screen
     */
    private void displayWelcomeScreen() {
        telemetry.clear();
        CalibrationUtils.addTelemetryHeader(telemetry, "UNIFIED CALIBRATION SYSTEM - MODULAR");
        
        telemetry.addLine("🎯 MODULAR CALIBRATION via FTC DASHBOARD");
        telemetry.addLine("");
        telemetry.addLine("📊 Connect to Dashboard: http://192.168.43.1:8080/dash");
        telemetry.addLine("");
        telemetry.addLine("🔧 AVAILABLE CALIBRATION MODES:");
        for (CalibrationMode mode : CalibrationMode.values()) {
            String status = "✅"; // All modules are now implemented
            telemetry.addLine(String.format("  %s %d: %s", status, mode.id, mode.name));
        }
        telemetry.addLine("");
        telemetry.addLine("⚙️ HOW TO USE:");
        telemetry.addLine("1. Select mode using dropdown: 0_ModeSelector → CALIBRATION_MODE");
        telemetry.addLine("2. Adjust parameters in real-time for selected mode");
        telemetry.addLine("3. Toggle ENABLE_TESTING to run calibration tests");
        telemetry.addLine("4. Observe performance metrics and recommendations");
        telemetry.addLine("5. WRITE DOWN optimized values from Dashboard");
        telemetry.addLine("6. Manually update source files per calibration manual");
        telemetry.addLine("");
        telemetry.addLine("🏗️ MODULAR ARCHITECTURE:");
        telemetry.addLine("• Each calibration is a separate module");
        telemetry.addLine("• Easy to add new calibrations");
        telemetry.addLine("• Maintainable and scalable design");
        telemetry.addLine("");
        telemetry.addLine("🚀 Press START when ready");
        telemetry.update();
    }
    
    /**
     * Update current mode from Dashboard
     */
    private void updateCurrentMode() {
        CalibrationMode newMode = CalibrationMode.fromId(_0_ModeSelector.CALIBRATION_MODE.id);
        if (newMode != currentMode) {
            lastMode = currentMode;
            currentMode = newMode;
        }
    }
    
    /**
     * Handle mode changes
     */
    private void handleModeChange() {
        if (currentMode != lastMode) {
            // Stop current calibration test
            if (currentCalibration != null && currentCalibration.isTestRunning()) {
                currentCalibration.stopCalibrationTest();
            }
            
            // Switch to new calibration module
            if (calibrationModules[currentMode.id] != null) {
                currentCalibration = calibrationModules[currentMode.id];
            } else {
                currentCalibration = null;
            }
            
            lastMode = currentMode;
        }
    }
    
    /**
     * Handle parameter reset requests
     */
    private void handleParameterReset() {
        if (_0_ModeSelector.RESET_TO_DEFAULTS) {
            if (currentCalibration != null) {
                currentCalibration.resetParameters();
            }
            _0_ModeSelector.RESET_TO_DEFAULTS = false;
        }
    }
    
    /**
     * Handle test control
     */
    private void handleTestControl() {
        if (currentCalibration == null) return;
        
        if (_0_ModeSelector.ENABLE_TESTING && !currentCalibration.isTestRunning()) {
            currentCalibration.startCalibrationTest();
        } else if (!_0_ModeSelector.ENABLE_TESTING && currentCalibration.isTestRunning()) {
            currentCalibration.stopCalibrationTest();
        }
    }
    
    /**
     * Update current calibration
     */
    private void updateCurrentCalibration() {
        if (currentCalibration != null) {
            currentCalibration.updateCalibrationTest();
        }
    }
    
    /**
     * Display status
     */
    private void displayStatus() {
        telemetry.clear();
        
        // Header with current mode
        CalibrationUtils.addTelemetryHeader(telemetry, 
            "UNIFIED CALIBRATION - " + currentMode.name.toUpperCase());
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Current Mode", "%d: %s", currentMode.id, currentMode.name);
        
        if (currentCalibration != null) {
            telemetry.addData("Test Status", currentCalibration.isTestRunning() ? "RUNNING" : "STOPPED");
            telemetry.addLine("");
            
            // Display calibration-specific status
            currentCalibration.displayStatus();
        } else {
            telemetry.addData("Status", "❌ Module initialization failed");
            telemetry.addLine("");
            telemetry.addLine("This calibration module failed to initialize.");
            telemetry.addLine("Check hardware connections and try again.");
        }
        
        // Common instructions
        telemetry.addLine("");
        telemetry.addLine("📊 Adjust parameters in FTC Dashboard");
        telemetry.addLine("🔄 Toggle ENABLE_TESTING to start/stop tests");
        telemetry.addLine("📝 Write down final values and update source files manually");
        
        // Dashboard reminder
        if (runtime.seconds() < 30) {
            telemetry.addLine("");
            telemetry.addLine("📊 Dashboard: http://192.168.43.1:8080/dash");
        }
    }
    
    /**
     * Stop all systems
     */
    private void stopAllSystems() {
        // Stop current calibration
        if (currentCalibration != null && currentCalibration.isTestRunning()) {
            currentCalibration.stopCalibrationTest();
        }
        
        // Stop motion system
        if (motionExecutor != null) {
            motionExecutor.stop();
        }
        
        // Stop all motors
        if (frontLeftDrive != null) frontLeftDrive.setPower(0);
        if (frontRightDrive != null) frontRightDrive.setPower(0);
        if (backLeftDrive != null) backLeftDrive.setPower(0);
        if (backRightDrive != null) backRightDrive.setPower(0);
    }
    
    /**
     * Get dashboard manager for calibration modules
     */
    public SmartDashboardManager getDashboardManager() {
        return dashboardManager;
    }
}

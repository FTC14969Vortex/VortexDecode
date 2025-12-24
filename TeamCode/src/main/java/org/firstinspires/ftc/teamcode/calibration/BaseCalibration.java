package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.calibration.CalibrationUtils.DataCollector;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.OdometryManager;

/**
 * Base class for all calibration modules
 * 
 * Provides common functionality:
 * - Hardware access (motors, motion system)
 * - Data collection and performance metrics
 * - Test timing and state management
 * - Telemetry utilities
 * 
 * Each calibration module extends this class and implements:
 * - initializeCalibration() - Setup calibration-specific state
 * - startTest() - Begin calibration test
 * - updateTest() - Update test execution (called at 20Hz)
 * - stopTest() - End test and calculate metrics
 * - displayStatus() - Show calibration-specific telemetry
 * - resetParameters() - Reset to default values
 */
public abstract class BaseCalibration {
    
    // ========== HARDWARE ACCESS ==========
    
    protected DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    protected MotionExecutor motionExecutor;
    protected OdometryManager odometryManager;
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Object parentOpMode;  // Reference to the parent OpMode (MotionCalibrationAndDemo)
    
    // ========== TEST STATE ==========
    
    protected boolean testRunning = false;
    protected ElapsedTime testTimer = new ElapsedTime();
    
    // ========== DATA COLLECTION ==========
    
    protected DataCollector dataCollector = new DataCollector(200);
    protected DataCollector targetDataCollector = new DataCollector(200);
    protected DataCollector errorDataCollector = new DataCollector(200);
    
    // ========== PERFORMANCE METRICS ==========
    
    protected double finalError = 0;
    protected double maxError = 0;
    protected double settlingTime = 0;
    protected double riseTime = 0;
    protected double overshoot = 0;
    protected double steadyStateError = 0;
    
    /**
     * Initialize the calibration module with hardware and telemetry
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry,
                          DcMotorEx frontLeft, DcMotorEx frontRight, 
                          DcMotorEx backLeft, DcMotorEx backRight,
                          MotionExecutor motionExecutor, OdometryManager odometryManager,
                          Object parentOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.frontLeftDrive = frontLeft;
        this.frontRightDrive = frontRight;
        this.backLeftDrive = backLeft;
        this.backRightDrive = backRight;
        this.motionExecutor = motionExecutor;
        this.odometryManager = odometryManager;
        this.parentOpMode = parentOpMode;
        
        // Initialize calibration-specific state
        initializeCalibration();
    }
    
    /**
     * Start the calibration test
     */
    public void startCalibrationTest() {
        if (testRunning) return;
        
        testRunning = true;
        testTimer.reset();
        resetDataCollectors();
        resetPerformanceMetrics();
        
        // Start calibration-specific test
        startTest();
    }
    
    /**
     * Stop the calibration test
     */
    public void stopCalibrationTest() {
        if (!testRunning) return;
        
        testRunning = false;
        
        // Stop calibration-specific test
        stopTest();
        
        // Stop all motors
        stopAllMotors();
        
        // Stop motion system
        if (motionExecutor != null) {
            motionExecutor.stop();
        }
    }
    
    /**
     * Update the calibration test (called at 20Hz)
     */
    public void updateCalibrationTest() {
        if (!testRunning) return;
        
        // Update calibration-specific test
        updateTest();
    }
    
    /**
     * Check if test is currently running
     */
    public boolean isTestRunning() {
        return testRunning;
    }
    
    /**
     * Get test elapsed time
     */
    public double getTestTime() {
        return testTimer.seconds();
    }
    
    /**
     * Reset data collectors
     */
    protected void resetDataCollectors() {
        dataCollector.reset();
        targetDataCollector.reset();
        errorDataCollector.reset();
    }
    
    /**
     * Reset performance metrics
     */
    protected void resetPerformanceMetrics() {
        finalError = 0;
        maxError = 0;
        settlingTime = 0;
        riseTime = 0;
        overshoot = 0;
        steadyStateError = 0;
    }
    
    /**
     * Stop all motors
     */
    protected void stopAllMotors() {
        if (frontLeftDrive != null) frontLeftDrive.setPower(0);
        if (frontRightDrive != null) frontRightDrive.setPower(0);
        if (backLeftDrive != null) backLeftDrive.setPower(0);
        if (backRightDrive != null) backRightDrive.setPower(0);
    }
    
    /**
     * Get motor by index (0=FL, 1=FR, 2=BL, 3=BR)
     */
    protected DcMotorEx getMotorByIndex(int index) {
        DcMotorEx[] motors = {frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};
        return motors[index % 4];
    }
    
    /**
     * Get motor name by index
     */
    protected String getMotorName(int index) {
        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
        return names[index % 4];
    }
    
    /**
     * Normalize angle to [-180, 180] range
     */
    protected double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    
    /**
     * Calculate velocity performance metrics
     */
    protected void calculateVelocityPerformanceMetrics(double targetVelocity) {
        if (dataCollector.getValues().isEmpty()) return;
        
        riseTime = CalibrationUtils.calculateRiseTime(dataCollector.getValues(), dataCollector.getTimes(), targetVelocity);
        settlingTime = CalibrationUtils.calculateSettlingTime(dataCollector.getValues(), dataCollector.getTimes(), targetVelocity);
        overshoot = CalibrationUtils.calculateOvershoot(dataCollector.getValues(), targetVelocity);
        steadyStateError = CalibrationUtils.calculateSteadyStateError(dataCollector.getValues(), targetVelocity);
    }
    
    /**
     * Calculate position performance metrics
     */
    protected void calculatePositionPerformanceMetrics(double targetPosition, double tolerance) {
        if (dataCollector.getValues().isEmpty()) return;
        
        double finalPosition = dataCollector.getValues().get(dataCollector.getValues().size() - 1);
        finalError = Math.abs(targetPosition - finalPosition);
        settlingTime = CalibrationUtils.calculateSettlingTime(errorDataCollector.getValues(), errorDataCollector.getTimes(), tolerance);
    }
    
    // ========== ABSTRACT METHODS (must be implemented by subclasses) ==========
    
    /**
     * Initialize calibration-specific state
     */
    protected abstract void initializeCalibration();
    
    /**
     * Start calibration-specific test
     */
    protected abstract void startTest();
    
    /**
     * Update calibration-specific test (called at 20Hz)
     */
    protected abstract void updateTest();
    
    /**
     * Stop calibration-specific test
     */
    protected abstract void stopTest();
    
    /**
     * Display calibration-specific telemetry
     */
    public abstract void displayStatus();
    
    /**
     * Reset calibration parameters to defaults
     */
    public abstract void resetParameters();
    
    /**
     * Get the name of this calibration
     */
    public abstract String getCalibrationName();
    
    /**
     * Get the description of this calibration
     */
    public abstract String getCalibrationDescription();
}

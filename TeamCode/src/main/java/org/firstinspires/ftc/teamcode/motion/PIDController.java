package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Full PID Controller implementation for motion control.
 * 
 * Provides proportional, integral, and derivative control with:
 * - Anti-windup protection for integral term
 * - Configurable output limits
 * - Acceleration limiting
 * - Circular value support (for heading control)
 * - Deadband and tolerance zones
 * 
 * This replaces the external ProportionalControl dependency and provides
 * enhanced control capabilities for precise robot motion.
 */
public class PIDController {
    
    // ========== PID GAINS ==========
    private double kP;  // Proportional gain
    private double kI;  // Integral gain  
    private double kD;  // Derivative gain
    
    // ========== CONTROL PARAMETERS ==========
    private double setpoint;
    private double tolerance;
    private double deadband;
    private double outputLimit;
    private double accelerationLimit;
    private boolean circular;  // For heading control (wraps +/-180 degrees)
    
    // ========== STATE VARIABLES ==========
    private double lastError;
    private double lastInput;  // For derivative on measurement
    private double integralSum;
    private double lastOutput;
    private boolean atTarget;
    private ElapsedTime timer;
    private boolean firstCall;  // Flag to handle first call timing
    
    // ========== ANTI-WINDUP PARAMETERS ==========
    private double integralLimit;
    private boolean enableAntiWindup;
    
    /**
     * Creates a new PID controller
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param outputLimit Maximum output magnitude
     * @param accelerationLimit Maximum rate of output change per second
     * @param tolerance Target tolerance for atTarget detection
     * @param deadband Deadband around setpoint (no output within this range)
     * @param circular True for circular values (heading), false for linear
     */
    public PIDController(double kP, double kI, double kD, double outputLimit, 
                        double accelerationLimit, double tolerance, double deadband, 
                        boolean circular) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.outputLimit = Math.abs(outputLimit);
        this.accelerationLimit = Math.abs(accelerationLimit);
        this.tolerance = Math.abs(tolerance);
        this.deadband = Math.abs(deadband);
        this.circular = circular;
        
        // Initialize state
        this.timer = new ElapsedTime();
        this.integralLimit = outputLimit * 0.5;  // Limit integral to 50% of max output
        this.enableAntiWindup = true;
        
        reset(0.0);
    }
    
    /**
     * Simplified constructor with default anti-windup settings
     */
    public PIDController(double kP, double kI, double kD, double outputLimit, 
                        double accelerationLimit, double tolerance, double deadband, 
                        boolean circular, double integralLimit) {
        this(kP, kI, kD, outputLimit, accelerationLimit, tolerance, deadband, circular);
        this.integralLimit = Math.abs(integralLimit);
    }
    
    /**
     * Calculates PID output based on current input value
     * 
     * @param input Current measured value (from sensors)
     * @return Control output value
     */
    public double calculate(double input) {
        double deltaTime = 0.0;
        
        // Handle first call - skip timing calculations
        if (firstCall) {
            firstCall = false;
            deltaTime = 0.0;  // No time-based calculations on first call
        } else {
            deltaTime = timer.seconds();
        }
        timer.reset();
        
        // Calculate error
        double error = setpoint - input;
        
        // Handle circular values (heading control)
        if (circular) {
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;
        }
        
        // Check if within tolerance
        atTarget = Math.abs(error) <= tolerance;
        
        // Apply deadband
        if (Math.abs(error) <= deadband) {
            // Within deadband - stop and reset integral
            integralSum = 0;
            lastError = error;
            lastOutput = 0;
            return 0;
        }
        
        // Calculate PID terms
        double proportional = kP * error;
        
        // Integral term with anti-windup
        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            
            // Anti-windup: clamp integral sum
            if (enableAntiWindup) {
                double maxIntegral = integralLimit / Math.abs(kI + 1e-6);  // Avoid division by zero
                integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
            }
        }
        double integral = kI * integralSum;
        
        // Derivative term (derivative on measurement to avoid derivative kick)
        double derivative = 0;
        if (deltaTime > 0) {
            derivative = -kD * (input - lastInput) / deltaTime;  // Negative because we want derivative of error
        }
        
        // Combine PID terms
        double output = proportional + integral + derivative;
        
        // Apply output limits
        output = Math.max(-outputLimit, Math.min(outputLimit, output));
        
        // Apply acceleration limiting
        if (deltaTime > 0 && accelerationLimit > 0) {
            double maxChange = accelerationLimit * deltaTime;
            double outputChange = output - lastOutput;
            
            if (outputChange > maxChange) {
                output = lastOutput + maxChange;
            } else if (outputChange < -maxChange) {
                output = lastOutput - maxChange;
            }
        }
        
        // Update state for next iteration
        lastError = error;
        lastInput = input;
        lastOutput = output;
        
        return output;
    }
    
    /**
     * Resets the controller state and sets new setpoint
     * 
     * @param newSetpoint New target value
     */
    public void reset(double newSetpoint) {
        this.setpoint = newSetpoint;
        this.lastError = 0;
        this.lastInput = 0;
        this.integralSum = 0;
        this.lastOutput = 0;
        this.atTarget = false;
        this.firstCall = true;  // Reset first call flag
        this.timer.reset();
    }
    
    /**
     * Resets only the controller state (keeps current setpoint)
     */
    public void reset() {
        reset(this.setpoint);
    }
    
    // ========== GETTERS AND SETTERS ==========
    
    /**
     * Checks if the controller is at the target
     * @return true if within tolerance
     */
    public boolean atTarget() {
        return atTarget;
    }
    
    /**
     * Gets the current setpoint
     * @return Current target value
     */
    public double getSetpoint() {
        return setpoint;
    }
    
    /**
     * Sets a new setpoint without resetting controller state
     * @param newSetpoint New target value
     */
    public void setSetpoint(double newSetpoint) {
        this.setpoint = newSetpoint;
    }
    
    /**
     * Gets the current error (setpoint - input from last calculate() call)
     * @return Current error value
     */
    public double getError() {
        return lastError;
    }
    
    /**
     * Gets the current integral sum
     * @return Accumulated integral value
     */
    public double getIntegralSum() {
        return integralSum;
    }
    
    /**
     * Gets the last output value
     * @return Last calculated output
     */
    public double getLastOutput() {
        return lastOutput;
    }
    
    // ========== TUNING METHODS ==========
    
    /**
     * Updates PID gains
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    /**
     * Gets the proportional gain
     * @return Current Kp value
     */
    public double getKp() {
        return kP;
    }
    
    /**
     * Sets the proportional gain
     * @param kP New proportional gain
     */
    public void setKp(double kP) {
        this.kP = kP;
    }
    
    /**
     * Gets the integral gain
     * @return Current Ki value
     */
    public double getKi() {
        return kI;
    }
    
    /**
     * Sets the integral gain
     * @param kI New integral gain
     */
    public void setKi(double kI) {
        this.kI = kI;
    }
    
    /**
     * Gets the derivative gain
     * @return Current Kd value
     */
    public double getKd() {
        return kD;
    }
    
    /**
     * Sets the derivative gain
     * @param kD New derivative gain
     */
    public void setKd(double kD) {
        this.kD = kD;
    }
    
    /**
     * Sets the output limit
     * @param limit Maximum output magnitude
     */
    public void setOutputLimit(double limit) {
        this.outputLimit = Math.abs(limit);
    }
    
    /**
     * Sets the acceleration limit
     * @param limit Maximum rate of output change per second
     */
    public void setAccelerationLimit(double limit) {
        this.accelerationLimit = Math.abs(limit);
    }
    
    /**
     * Sets tolerance and deadband
     * @param tolerance Target tolerance for atTarget detection
     * @param deadband Deadband around setpoint
     */
    public void setTolerances(double tolerance, double deadband) {
        this.tolerance = Math.abs(tolerance);
        this.deadband = Math.abs(deadband);
    }
    
    /**
     * Enables or disables anti-windup protection
     * @param enable True to enable anti-windup
     */
    public void setAntiWindup(boolean enable) {
        this.enableAntiWindup = enable;
        if (!enable) {
            integralSum = 0;  // Clear integral when disabling
        }
    }
    
    /**
     * Sets the integral windup limit
     * @param limit Maximum integral contribution to output
     */
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }
    
    // ========== FACTORY METHODS ==========
    
    /**
     * Creates a PID controller configured for X-axis position control
     * Uses velocity-based output limits for proper motion control
     * @return X-axis position controller with settings from MotionConfig
     */
    public static PIDController createPositionXController() {
        return new PIDController(
            MotionConfig.POSITION_X_KP,
            MotionConfig.POSITION_X_KI,
            MotionConfig.POSITION_X_KD,
            MotionConfig.MAX_LINEAR_VELOCITY,      // Use velocity limit instead of power limit
            MotionConfig.MAX_LINEAR_ACCELERATION,  // Use acceleration limit instead of power ramp
            MotionConfig.POSITION_TOLERANCE,
            MotionConfig.POSITION_DEADBAND,
            false  // Not circular
        );
    }
    
    /**
     * Creates a PID controller configured for Y-axis position control
     * Uses velocity-based output limits for proper motion control
     * @return Y-axis position controller with settings from MotionConfig
     */
    public static PIDController createPositionYController() {
        return new PIDController(
            MotionConfig.POSITION_Y_KP,
            MotionConfig.POSITION_Y_KI,
            MotionConfig.POSITION_Y_KD,
            MotionConfig.MAX_LINEAR_VELOCITY,      // Use velocity limit instead of power limit
            MotionConfig.MAX_LINEAR_ACCELERATION,  // Use acceleration limit instead of power ramp
            MotionConfig.POSITION_TOLERANCE,
            MotionConfig.POSITION_DEADBAND,
            false  // Not circular
        );
    }
    
    /**
     * Creates a PID controller configured for distance control (2-PID architecture)
     * Uses velocity-based output limits for unified linear motion control
     * @return Distance controller with settings from MotionConfig
     */
    public static PIDController createDistanceController() {
        return new PIDController(
            MotionConfig.DISTANCE_KP,
            MotionConfig.DISTANCE_KI,
            MotionConfig.DISTANCE_KD,
            MotionConfig.MAX_LINEAR_VELOCITY,      // Use velocity limit instead of power limit
            MotionConfig.MAX_LINEAR_ACCELERATION,  // Use acceleration limit instead of power ramp
            MotionConfig.POSITION_TOLERANCE,
            MotionConfig.POSITION_DEADBAND,
            false  // Not circular
        );
    }
    
    /**
     * Creates a PID controller configured for position control (legacy)
     * Uses velocity-based output limits for proper motion control
     * @return Position controller with settings from MotionConfig
     * @deprecated Use createPositionXController() or createPositionYController() instead
     */
    @Deprecated
    public static PIDController createPositionController() {
        return new PIDController(
            MotionConfig.POSITION_X_KP,
            MotionConfig.POSITION_X_KI,
            MotionConfig.POSITION_X_KD,
            MotionConfig.MAX_LINEAR_VELOCITY,      // Use velocity limit instead of power limit
            MotionConfig.MAX_LINEAR_ACCELERATION,  // Use acceleration limit instead of power ramp
            MotionConfig.POSITION_TOLERANCE,
            MotionConfig.POSITION_DEADBAND,
            false  // Not circular
        );
    }
    
    /**
     * Creates a PID controller configured for heading control
     * Uses velocity-based output limits for proper motion control
     * @return Heading controller with settings from MotionConfig
     */
    public static PIDController createHeadingController() {
        return new PIDController(
            MotionConfig.HEADING_KP,
            MotionConfig.HEADING_KI,
            MotionConfig.HEADING_KD,
            MotionConfig.MAX_ANGULAR_VELOCITY,     // Use angular velocity limit instead of power limit
            MotionConfig.MAX_ANGULAR_ACCELERATION, // Use angular acceleration limit instead of power ramp
            MotionConfig.HEADING_TOLERANCE,
            MotionConfig.HEADING_DEADBAND,
            true  // Circular (wraps +/-180 degrees)
        );
    }
    
    /**
     * Creates a custom PID controller
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param outputLimit Maximum output
     * @param circular True for circular values
     * @return Configured PID controller
     */
    public static PIDController createCustomController(double kP, double kI, double kD, 
                                                      double outputLimit, boolean circular) {
        return new PIDController(kP, kI, kD, outputLimit, 
                               MotionConfig.MAX_LINEAR_ACCELERATION,
                               circular ? MotionConfig.HEADING_TOLERANCE : MotionConfig.POSITION_TOLERANCE,
                               circular ? MotionConfig.HEADING_DEADBAND : MotionConfig.POSITION_DEADBAND,
                               circular);
    }
    
    @Override
    public String toString() {
        return String.format("PID[P=%.3f, I=%.3f, D=%.3f, setpoint=%.2f, error=%.2f, output=%.2f, atTarget=%s]",
            kP, kI, kD, setpoint, lastError, lastOutput, atTarget);
    }
}

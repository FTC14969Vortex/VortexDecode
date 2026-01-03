package org.firstinspires.ftc.teamcode.calibration;

/**
 * Calibration coefficients derived from measurement and testing.
 * 
 * These coefficients correct for differences between theoretical/geometric values
 * and actual robot performance. They are determined through systematic testing
 * and measurement procedures.
 * 
 * CALIBRATION PROCEDURE:
 * 1. Start with geometric measurements in RobotGeometry and RobotConstants
 * 2. Run calibration tests using odometry vs real-world measurements
 * 3. Calculate correction factors based on odometry readings vs actual results
 * 4. Update the coefficients below
 * 5. Re-test to verify improved accuracy
 * 
 * IMPORTANT: These coefficients are robot-specific and may change with:
 * - Wheel wear
 * - Mechanical adjustments
 * - Battery condition
 * - Field surface changes
 * - Odometry sensor mounting changes
 */
public class CalibrationCoefficients {
    
    // ========== ODOMETRY CALIBRATION ==========
    
    /**
     * Odometry X-axis scaling factor
     * Corrects for systematic errors in X-direction odometry readings
     * 
     * MEASUREMENT PROCEDURE:
     * 1. Execute long straight-line moves in +X and -X directions (e.g., 48" forward/backward)
     * 2. Compare odometry readings to measured distances with tape measure
     * 3. Calculate scaling factor: scale = actual_distance / odometry_distance
     * 4. Example: If odometry reports 48" but actual is 46.5", scale = 46.5/48 = 0.969
     * 
     * Applied in OdometryManager to correct X-axis readings
     */
    public static final double ODOMETRY_X_SCALE = 1.00;  // TODO: CALIBRATE WITH OdometryCalibrationModule
    
    /**
     * Odometry Y-axis scaling factor
     * Corrects for systematic errors in Y-direction odometry readings
     * 
     * MEASUREMENT PROCEDURE:
     * 1. Execute long strafing moves in +Y and -Y directions (e.g., 36" left/right)
     * 2. Compare odometry readings to measured distances with tape measure
     * 3. Calculate scaling factor: scale = actual_distance / odometry_distance
     * 4. Example: If odometry reports 36" but actual is 37.2", scale = 37.2/36 = 1.033
     * 
     * Applied in OdometryManager to correct Y-axis readings
     */
    public static final double ODOMETRY_Y_SCALE = 1.00;  // TODO: CALIBRATE WITH OdometryCalibrationModule
    
    /**
     * Odometry heading scaling factor
     * Corrects for systematic errors in heading measurements
     * 
     * MEASUREMENT PROCEDURE:
     * 1. Execute large rotations (e.g., 360 degrees, 720 degrees) and compare odometry heading to compass/protractor
     * 2. Calculate scaling factor: scale = actual_angle / odometry_angle
     * 3. Example: If odometry reports 360 degrees but actual is 355 degrees, scale = 355/360 = 0.986
     * 
     * Applied in OdometryManager to correct heading readings
     */
    public static final double ODOMETRY_HEADING_SCALE = 1.00;  // TODO: CALIBRATE WITH OdometryCalibrationModule
    
    // ========== GEOMETRY VALUES (CALIBRATED) ==========
    // NOTE: CALIBRATED_WHEEL_DIAMETER, CALIBRATED_TRACK_WIDTH, and CALIBRATED_WHEELBASE have been removed.
    // Use RobotConstants.WHEEL_DIAMETER, RobotConstants.TRACK_WIDTH, and RobotConstants.WHEELBASE directly.
    
    // ========== PERFORMANCE CONSTANTS ==========
    

    
    // ========== VELOCITY LIMITS (CALIBRATED VALUES) ==========
    
    /**
     * Maximum linear velocity (inches/sec)
     
     * Current calculation: Based on motor specs and wheel geometry
     * Future: Will be determined through slip detection and performance testing
     */
    public static final double CALIBRATED_MAX_LINEAR_VELOCITY =  RobotConstants.MAX_THEORETICAL_LINEAR_VELOCITY;  // Conservative 70% of theoretical ~ 47 in/s
    
    /**
     * Maximum angular velocity (degrees/sec)     
     * 
     * Current calculation: Based on linear velocity and drivetrain geometry
     * Future: Will be determined through rotation testing and stability analysis
     */
    public static final double CALIBRATED_MAX_ANGULAR_VELOCITY = RobotConstants.MAX_THEORETICAL_ANGULAR_VELOCITY;
    
    /**
     * Maximum linear acceleration (inches/sec^2)
     * 
     * DEVELOPMENT STATUS: Currently conservative estimate, needs empirical validation
     * TODO: Implement VelocityLimitsCalibrationModule to determine actual limits
     * 
     * Current value: Conservative estimate to prevent wheel slip
     * Future: Will be determined through acceleration testing and slip detection
     */
    public static final double CALIBRATED_MAX_LINEAR_ACCELERATION = 100.0;  // was 50, too slow
    
    /**
     * Maximum angular acceleration (degrees/sec^2)
     * 
     * DEVELOPMENT STATUS: Currently conservative estimate, needs empirical validation
     * TODO: Implement VelocityLimitsCalibrationModule to determine actual limits
     * 
     * Current value: Conservative estimate to prevent instability
     * Future: Will be determined through rotational acceleration testing
     */
    public static final double CALIBRATED_MAX_ANGULAR_ACCELERATION = 180.0;  //
    
    // ========== DISTANCE SCALING FACTORS ==========
    // NOTE: FORWARD_DISTANCE_SCALE and STRAFE_DISTANCE_SCALE have been removed.
    // Use ODOMETRY_X_SCALE and ODOMETRY_Y_SCALE instead for distance corrections.
    
    // ========== 8-FACTOR WHEEL VELOCITY SCALING ==========
    
    /**
     * Directional wheel velocity scaling factors from odometry-based calibration
     * These factors are applied ONLY to the kinematic matrix, not directly to motors
     * 
     * Each wheel has separate scaling for positive and negative contributions
     * to account for directional performance differences in mecanum wheels
     */
    
    // teste at 40 inch/s close to 70% of theoretical, duration 1s
    // Positive contribution scaling factors (wheel spinning forward)
    public static double WHEEL_VELOCITY_SCALE_FL_POSITIVE = 1.04;  // Front Left forward
    public static double WHEEL_VELOCITY_SCALE_FR_POSITIVE = 1.05;  // Front Right forward
    public static double WHEEL_VELOCITY_SCALE_BL_POSITIVE = 1.01;  // Back Left forward
    public static double WHEEL_VELOCITY_SCALE_BR_POSITIVE = 1.03;  // Back Right forward

    // Negative contribution scaling factors (wheel spinning backward)
    public static double WHEEL_VELOCITY_SCALE_FL_NEGATIVE = 1.01;  // Front Left backward
    public static double WHEEL_VELOCITY_SCALE_FR_NEGATIVE = 1.03;  // Front Right backward
    public static double WHEEL_VELOCITY_SCALE_BL_NEGATIVE = 1.03;  // Back Left backward
    public static double WHEEL_VELOCITY_SCALE_BR_NEGATIVE = 1.05;  // Back Right backward

    // 8-Factor calibration control and metadata
    public static boolean USE_8_FACTOR_VELOCITY_SCALING = true;  // Flag: use 8-factor scaling OR kinematic matrix
    public static boolean WHEEL_VELOCITY_SCALES_CALIBRATED = true;
    public static String WHEEL_VELOCITY_CALIBRATION_DATE = "12/24/2025";
    public static double WHEEL_VELOCITY_CALIBRATION_ACCURACY = 999.0;  // RMS error in in/s

    // ========== CALIBRATION HISTORY ==========
    
    /**
     * Date of last calibration (for tracking)
     */
    public static final String LAST_CALIBRATION_DATE = "2025-11-25";  // TODO: UPDATE WHEN CALIBRATED!
    
    /**
     * Notes about calibration conditions
     */
    public static final String CALIBRATION_NOTES = 
        "Initial values - not yet calibrated. Update after running calibration procedures.";  // TODO: UPDATE!
    
    /**
     * Calibration accuracy achieved (position error in inches)
     */
    public static final double CALIBRATION_ACCURACY = 999.0;  // TODO: UPDATE AFTER CALIBRATION!
    
    // ========== 3x4 KINEMATIC CALIBRATION MATRIX ==========
    
    /**
     * 3x4 Kinematic calibration matrix for cross-coupling correction
     * 
     * [vx_actual]     [K11  K12  K13  K14] [wFL]
     * [vy_actual]  =  [K21  K22  K23  K24] [wFR]
     * [w_actual ]     [K31  K32  K33  K34] [wBL]
     *                                      [wBR]
     * 
     * Calibrated through systematic motion testing with combined translation and rotation.
     */
    
    // Row 1: vx coefficients (forward/backward motion)
    public static double KINEMATIC_K11 = 0.25;  // Front-left contribution to vx
    public static double KINEMATIC_K12 = 0.25;  // Front-right contribution to vx
    public static double KINEMATIC_K13 = 0.25;  // Back-left contribution to vx
    public static double KINEMATIC_K14 = 0.25;  // Back-right contribution to vx
    
    // Row 2: vy coefficients (left/right motion)
    public static double KINEMATIC_K21 = 0.25;   // Front-left contribution to vy
    public static double KINEMATIC_K22 = -0.25;  // Front-right contribution to vy
    public static double KINEMATIC_K23 = -0.25;  // Back-left contribution to vy
    public static double KINEMATIC_K24 = 0.25;   // Back-right contribution to vy
    
    // Row 3: w coefficients (rotational motion)
    private static final double IDEAL_OMEGA_COEFF = 1.0 / (4.0 * Math.sqrt(
        Math.pow(RobotConstants.TRACK_WIDTH / 2.0, 2) + 
        Math.pow(RobotConstants.WHEELBASE / 2.0, 2)));
    
    public static double KINEMATIC_K31 = IDEAL_OMEGA_COEFF;   // Front-left contribution to w
    public static double KINEMATIC_K32 = -IDEAL_OMEGA_COEFF;  // Front-right contribution to w
    public static double KINEMATIC_K33 = IDEAL_OMEGA_COEFF;   // Back-left contribution to w
    public static double KINEMATIC_K34 = -IDEAL_OMEGA_COEFF;  // Back-right contribution to w
    
    /**
     * Calibration metadata
     */
    public static boolean KINEMATIC_MATRIX_CALIBRATED = false;
    public static String KINEMATIC_CALIBRATION_DATE = "Not calibrated";
    public static double KINEMATIC_CALIBRATION_R_SQUARED = 0.0;  // Model fit quality
    public static double KINEMATIC_CALIBRATION_CONDITION_NUMBER = 0.0;  // Matrix conditioning
    public static double KINEMATIC_CALIBRATION_RMS_RESIDUAL = 0.0;  // Residual error
    public static int KINEMATIC_CALIBRATION_DATA_POINTS = 0;  // Number of data points used
    
    /**
     * Get the complete 3×4 kinematic matrix
     */
    public static double[][] getKinematicMatrix() {
        return new double[][] {
            {KINEMATIC_K11, KINEMATIC_K12, KINEMATIC_K13, KINEMATIC_K14},
            {KINEMATIC_K21, KINEMATIC_K22, KINEMATIC_K23, KINEMATIC_K24},
            {KINEMATIC_K31, KINEMATIC_K32, KINEMATIC_K33, KINEMATIC_K34}
        };
    }
    
    /**
     * Set the complete 3×4 kinematic matrix
     * 
     * IMPORTANT: When kinematic matrix is calibrated directly, reset 8-factor scaling
     * to avoid double-scaling. Matrix calibration takes precedence.
     */
    public static void setKinematicMatrix(double[][] matrix, double rSquared, 
                                         double conditionNumber, double rmsResidual, int dataPoints) {
        if (matrix.length != 3 || matrix[0].length != 4) {
            throw new IllegalArgumentException("Matrix must be 3×4");
        }
        
        // Row 1: vx coefficients
        KINEMATIC_K11 = matrix[0][0];
        KINEMATIC_K12 = matrix[0][1];
        KINEMATIC_K13 = matrix[0][2];
        KINEMATIC_K14 = matrix[0][3];
        
        // Row 2: vy coefficients
        KINEMATIC_K21 = matrix[1][0];
        KINEMATIC_K22 = matrix[1][1];
        KINEMATIC_K23 = matrix[1][2];
        KINEMATIC_K24 = matrix[1][3];
        
        // Row 3: ω coefficients
        KINEMATIC_K31 = matrix[2][0];
        KINEMATIC_K32 = matrix[2][1];
        KINEMATIC_K33 = matrix[2][2];
        KINEMATIC_K34 = matrix[2][3];
        
        // When kinematic matrix is directly calibrated, switch to matrix mode
        // This ensures clean separation between calibration methods
        USE_8_FACTOR_VELOCITY_SCALING = true;  // Use kinematic matrix calibration
        
        // Update metadata
        KINEMATIC_MATRIX_CALIBRATED = false;
        KINEMATIC_CALIBRATION_R_SQUARED = rSquared;
        KINEMATIC_CALIBRATION_CONDITION_NUMBER = conditionNumber;
        KINEMATIC_CALIBRATION_RMS_RESIDUAL = rmsResidual;
        KINEMATIC_CALIBRATION_DATA_POINTS = dataPoints;
        KINEMATIC_CALIBRATION_DATE = new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new java.util.Date());
    }
    
    /**
     * Get the effective kinematic matrix based on calibration mode
     * 
     * NOTE: This method is for FORWARD KINEMATICS ONLY (wheel vel → robot vel)
     * For 8-factor scaling in INVERSE kinematics, use robotVelocitiesToWheelVelocities8Factor() directly
     * 
     * AUTOMATIC CALIBRATION SELECTION:
     * - If USE_8_FACTOR_VELOCITY_SCALING = true: Returns ideal matrix (scaling applied elsewhere)
     * - If USE_8_FACTOR_VELOCITY_SCALING = false: Returns calibrated kinematic matrix
     * 
     * This provides automatic switching between calibration methods.
     */
    public static double[][] getEffectiveKinematicMatrix() {
        if (USE_8_FACTOR_VELOCITY_SCALING) {
            // For 8-factor mode: Return ideal matrix
            // The 8-factor scaling is applied during INVERSE kinematics, not here
            // This matrix is only used for forward kinematics (wheel → robot)
            return getIdealKinematicMatrix();
        } else {
            // Use calibrated kinematic matrix for full cross-coupling correction
            return getKinematicMatrix();
        }
    }
    
    /**
     * Get the ideal (theoretical) kinematic matrix
     * Used as baseline for 8-factor scaling approach
     */
    private static double[][] getIdealKinematicMatrix() {
        double[][] matrix = new double[3][4];
        
        // Row 1: vx coefficients (forward/backward motion)
        matrix[0][0] = 0.25;
        matrix[0][1] = 0.25;
        matrix[0][2] = 0.25;
        matrix[0][3] = 0.25;
        
        // Row 2: vy coefficients (left/right motion)
        matrix[1][0] = 0.25;
        matrix[1][1] = -0.25;
        matrix[1][2] = -0.25;
        matrix[1][3] = 0.25;
        
        // Row 3: omega coefficients (rotational motion)
        matrix[2][0] = IDEAL_OMEGA_COEFF;
        matrix[2][1] = -IDEAL_OMEGA_COEFF;
        matrix[2][2] = IDEAL_OMEGA_COEFF;
        matrix[2][3] = -IDEAL_OMEGA_COEFF;
        
        return matrix;
    }

    /**
     * Reset 8-factor scaling to defaults and reset kinematic matrix
     */
    public static void reset8FactorScaling() {
        WHEEL_VELOCITY_SCALE_FL_POSITIVE = 1.0;
        WHEEL_VELOCITY_SCALE_FR_POSITIVE = 1.0;
        WHEEL_VELOCITY_SCALE_BL_POSITIVE = 1.0;
        WHEEL_VELOCITY_SCALE_BR_POSITIVE = 1.0;
        
        WHEEL_VELOCITY_SCALE_FL_NEGATIVE = 1.0;
        WHEEL_VELOCITY_SCALE_FR_NEGATIVE = 1.0;
        WHEEL_VELOCITY_SCALE_BL_NEGATIVE = 1.0;
        WHEEL_VELOCITY_SCALE_BR_NEGATIVE = 1.0;
        
        WHEEL_VELOCITY_SCALES_CALIBRATED = false;
        WHEEL_VELOCITY_CALIBRATION_DATE = "Reset to defaults";
        WHEEL_VELOCITY_CALIBRATION_ACCURACY = 999.0;
        
        // Reset kinematic matrix to ideal values
        resetKinematicMatrix();
    }

    /**
     * Reset kinematic matrix to ideal values
     */
    public static void resetKinematicMatrix() {
        KINEMATIC_K11 = 0.25;
        KINEMATIC_K12 = 0.25;
        KINEMATIC_K13 = 0.25;
        KINEMATIC_K14 = 0.25;
        
        KINEMATIC_K21 = 0.25;
        KINEMATIC_K22 = -0.25;
        KINEMATIC_K23 = -0.25;
        KINEMATIC_K24 = 0.25;
        
        KINEMATIC_K31 = IDEAL_OMEGA_COEFF;
        KINEMATIC_K32 = -IDEAL_OMEGA_COEFF;
        KINEMATIC_K33 = IDEAL_OMEGA_COEFF;
        KINEMATIC_K34 = -IDEAL_OMEGA_COEFF;
        
        KINEMATIC_MATRIX_CALIBRATED = false;
        KINEMATIC_CALIBRATION_DATE = "Reset to ideal values";
        KINEMATIC_CALIBRATION_R_SQUARED = 0.0;
        KINEMATIC_CALIBRATION_CONDITION_NUMBER = 0.0;
        KINEMATIC_CALIBRATION_RMS_RESIDUAL = 0.0;
        KINEMATIC_CALIBRATION_DATA_POINTS = 0;
    }
    
    // ========== VALIDATION METHODS ==========
    
    /**
     * Validates calibration coefficients for reasonableness
     * 
     * @return ValidationResult with any issues found
     */
    public static ValidationResult validateCoefficients() {
        ValidationResult result = new ValidationResult();
        
        // Check odometry scaling factors
        double[] odometryScales = {ODOMETRY_X_SCALE, ODOMETRY_Y_SCALE, ODOMETRY_HEADING_SCALE};
        String[] scaleNames = {"X-axis", "Y-axis", "Heading"};
        
        for (int i = 0; i < odometryScales.length; i++) {
            if (odometryScales[i] < 0.95 || odometryScales[i] > 1.05) {
                result.addWarning("Odometry " + scaleNames[i] + " scale (" + odometryScales[i] + 
                                ") is outside typical range (0.95 to 1.05)");
            }
        }
        
        // Check if calibration has been performed
        if (CALIBRATION_ACCURACY > 10.0) {
            result.addWarning("Calibration accuracy (" + CALIBRATION_ACCURACY + 
                            "\") indicates calibration may not have been performed yet.");
        }
        
        // Performance constants validation removed (constants were removed)
        
        // Check velocity limits are reasonable
        if (CALIBRATED_MAX_LINEAR_VELOCITY > 80.0) {
            result.addWarning("Maximum linear velocity (" + String.format("%.1f", CALIBRATED_MAX_LINEAR_VELOCITY) + 
                            " in/s) is very high. Consider empirical validation.");
        }
        
        if (CALIBRATED_MAX_ANGULAR_VELOCITY > 360.0) {
            result.addWarning("Maximum angular velocity (" + String.format("%.1f", CALIBRATED_MAX_ANGULAR_VELOCITY) + 
                            " deg/s) is very high. Consider empirical validation.");
        }
        
        return result;
    }
    
    /**
     * Returns a summary of calibration coefficients
     * 
     * @return Formatted string with calibration information
     */
    public static String getCalibrationSummary() {
        StringBuilder summary = new StringBuilder();
        summary.append("=== CALIBRATION COEFFICIENTS SUMMARY ===\n");
        summary.append(String.format("Last Calibration: %s\n", LAST_CALIBRATION_DATE));
        summary.append(String.format("Calibration Accuracy: %.2f inches\n", CALIBRATION_ACCURACY));
        
        summary.append("\nOdometry Scaling Factors:\n");
        summary.append(String.format("  X-axis: %.4f (%.1f%% adjustment)\n", 
                                    ODOMETRY_X_SCALE, (ODOMETRY_X_SCALE - 1.0) * 100));
        summary.append(String.format("  Y-axis: %.4f (%.1f%% adjustment)\n", 
                                    ODOMETRY_Y_SCALE, (ODOMETRY_Y_SCALE - 1.0) * 100));
        summary.append(String.format("  Heading: %.4f (%.1f%% adjustment)\n", 
                                    ODOMETRY_HEADING_SCALE, (ODOMETRY_HEADING_SCALE - 1.0) * 100));
        
        summary.append("\nGeometry Values (from RobotConstants):\n");
        summary.append(String.format("  Wheel Diameter: %.4f inches\n", RobotConstants.WHEEL_DIAMETER));
        summary.append(String.format("  Track Width: %.4f inches\n", RobotConstants.TRACK_WIDTH));
        summary.append(String.format("  Wheelbase: %.4f inches\n", RobotConstants.WHEELBASE));
        
        summary.append("\nVelocity Limits (Development Status):\n");
        summary.append(String.format("  Max Linear Velocity: %.1f in/s (theoretical)\n", CALIBRATED_MAX_LINEAR_VELOCITY));
        summary.append(String.format("  Max Angular Velocity: %.1f deg/s (theoretical)\n", CALIBRATED_MAX_ANGULAR_VELOCITY));
        summary.append(String.format("  Max Linear Acceleration: %.1f in/s² (conservative)\n", CALIBRATED_MAX_LINEAR_ACCELERATION));
        summary.append(String.format("  Max Angular Acceleration: %.1f deg/s² (conservative)\n", CALIBRATED_MAX_ANGULAR_ACCELERATION));
        
        summary.append("\nVelocity Calibration Mode:\n");
        summary.append(String.format("  Active Mode: %s\n", USE_8_FACTOR_VELOCITY_SCALING ? "8-Factor Scaling" : "Kinematic Matrix"));
        summary.append(String.format("  8-Factor Calibrated: %s\n", WHEEL_VELOCITY_SCALES_CALIBRATED ? "Yes" : "No"));
        summary.append(String.format("  8-Factor Date: %s\n", WHEEL_VELOCITY_CALIBRATION_DATE));
        summary.append(String.format("  8-Factor Accuracy: %.3f in/s RMS\n", WHEEL_VELOCITY_CALIBRATION_ACCURACY));
        
        summary.append("\nNotes: ").append(CALIBRATION_NOTES);
        
        return summary.toString();
    }
    
    // ========== CALIBRATION TEST RESULTS ==========
    
    /**
     * Stores results from distance calibration tests
     */
    public static class DistanceTestResult {
        public double commandedDistance;
        public double measuredDistance;
        public double error;
        public double errorPercent;
        
        public DistanceTestResult(double commanded, double measured) {
            this.commandedDistance = commanded;
            this.measuredDistance = measured;
            this.error = measured - commanded;
            this.errorPercent = (error / commanded) * 100.0;
        }
        
        @Override
        public String toString() {
            return String.format("Commanded: %.1f\", Measured: %.1f\", Error: %.2f\" (%.1f%%)", 
                               commandedDistance, measuredDistance, error, errorPercent);
        }
    }
    
    /**
     * Stores results from rotation calibration tests
     */
    public static class RotationTestResult {
        public double commandedAngle;
        public double measuredAngle;
        public double error;
        public double errorPercent;
        
        public RotationTestResult(double commanded, double measured) {
            this.commandedAngle = commanded;
            this.measuredAngle = measured;
            this.error = measured - commanded;
            this.errorPercent = (error / commanded) * 100.0;
        }
        
        @Override
        public String toString() {
            return String.format("Commanded: %.1f deg, Measured: %.1f deg, Error: %.2f deg (%.1f%%)", 
                               commandedAngle, measuredAngle, error, errorPercent);
        }
    }
    
    // ========== VALIDATION RESULT CLASS ==========
    
    /**
     * Result of calibration coefficients validation
     */
    public static class ValidationResult {
        private StringBuilder errors = new StringBuilder();
        private StringBuilder warnings = new StringBuilder();
        private int errorCount = 0;
        private int warningCount = 0;
        
        public void addError(String error) {
            errors.append("ERROR: ").append(error).append("\n");
            errorCount++;
        }
        
        public void addWarning(String warning) {
            warnings.append("WARNING: ").append(warning).append("\n");
            warningCount++;
        }
        
        public boolean hasErrors() {
            return errorCount > 0;
        }
        
        public boolean hasWarnings() {
            return warningCount > 0;
        }
        
        public int getErrorCount() {
            return errorCount;
        }
        
        public int getWarningCount() {
            return warningCount;
        }
        
        @Override
        public String toString() {
            StringBuilder result = new StringBuilder();
            result.append("=== CALIBRATION COEFFICIENTS VALIDATION ===\n");
            result.append(String.format("Errors: %d, Warnings: %d\n", errorCount, warningCount));
            
            if (errorCount > 0) {
                result.append("\nERRORS:\n").append(errors);
            }
            
            if (warningCount > 0) {
                result.append("\nWARNINGS:\n").append(warnings);
            }
            
            if (errorCount == 0 && warningCount == 0) {
                result.append("\n[OK] All calibration coefficients appear valid!");
            }
            
            return result.toString();
        }
    }
}

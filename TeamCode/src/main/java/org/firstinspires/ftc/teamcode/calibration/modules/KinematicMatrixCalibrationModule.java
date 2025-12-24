package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.MecanumKinematics;
import org.firstinspires.ftc.teamcode.motion.MotionState;
import org.firstinspires.ftc.teamcode.motion.OdometryManager;

import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

// EJML imports for robust SVD-based matrix calculations
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

/**
 * Calibrates the 3×4 kinematic matrix for cross-coupling correction
 * 
 * Enhanced approach with combined translation and rotation patterns:
 * - Uses time-based velocity measurement (3 seconds per pattern)
 * - Includes rotation in calibration patterns for complete matrix calibration
 * - Measures commanded wheel velocities vs actual robot motion
 * - Uses least squares to solve for 12 calibration coefficients
 * 
 * Mathematical Model:
 * [vx_actual]     [K11  K12  K13  K14] [ωFL]
 * [vy_actual]  =  [K21  K22  K23  K24] [ωFR]
 * [ω_actual ]     [K31  K32  K33  K34] [ωBL]
 *                                      [ωBR]
 */
public class KinematicMatrixCalibrationModule extends BaseCalibration {
    
    // ========== DASHBOARD PARAMETERS ==========
    
    @Config
    public static class _8_KinematicMatrix {
        public static boolean ENABLE_CALIBRATION = false;  // Enable/disable calibration test
        public static double MEASUREMENT_TIME = 3.0;  // seconds - time to measure each pattern
        public static double STEADY_STATE_WAIT_TIME = 0.5;  // seconds - wait for steady state
        public static double CALIBRATION_LINEAR_VELOCITY_SCALE = 0.7;  // 70% of max velocity for stability
        public static double CALIBRATION_ANGULAR_VELOCITY_SCALE = 0.7;  // 70% of max angular velocity
        public static double MIN_R_SQUARED = 0.90;  // Minimum R² for good calibration
        public static double MAX_CONDITION_NUMBER = 100.0;  // Maximum condition number
        public static double MAX_RMS_RESIDUAL = 1.0;  // Maximum RMS residual
    }
    
    // Calibration velocity parameters - calculated from Dashboard settings
    private double calibrationLinearVelocity;
    private double calibrationAngularVelocity;
    
    // Data collection
    private List<CalibrationDataPoint> calibrationData;
    
    public KinematicMatrixCalibrationModule() {
        // BaseCalibration constructor takes no arguments
        this.calibrationData = new ArrayList<>();
    }
    
    // ========== ABSTRACT METHOD IMPLEMENTATIONS ==========
    
    @Override
    protected void initializeCalibration() {
        // Initialize calibration-specific state
        calibrationData.clear();
        
        // Calculate calibration velocities from Dashboard settings
        this.calibrationLinearVelocity = CalibrationCoefficients.CALIBRATED_MAX_LINEAR_VELOCITY * _8_KinematicMatrix.CALIBRATION_LINEAR_VELOCITY_SCALE;
        this.calibrationAngularVelocity = CalibrationCoefficients.CALIBRATED_MAX_ANGULAR_VELOCITY * _8_KinematicMatrix.CALIBRATION_ANGULAR_VELOCITY_SCALE;
    }
    
    @Override
    protected void startTest() {
        // Start calibration test only if enabled via Dashboard
        if (_8_KinematicMatrix.ENABLE_CALIBRATION) {
            runCalibration();
        }
    }
    
    @Override
    protected void updateTest() {
        // Update test execution (called at 20Hz)
        // For this calibration, all work is done in startTest()
    }
    
    @Override
    protected void stopTest() {
        // Stop calibration test
        // Stop all motion
        if (motionExecutor != null) {
            motionExecutor.stop();
        }
    }
    
    @Override
    public void displayStatus() {
        // Display calibration-specific telemetry
        telemetry.addData("Calibration", "Kinematic Matrix Calibration");
        telemetry.addData("Data Points", calibrationData.size());
        telemetry.addData("Status", testRunning ? "Running" : "Stopped");
        telemetry.update();
    }
    
    @Override
    public void resetParameters() {
        // Reset calibration parameters to defaults
        calibrationData.clear();
    }
    
    @Override
    public String getCalibrationName() {
        return "Kinematic Matrix Calibration";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Calibrates the 3×4 kinematic matrix for cross-coupling correction using 18 motion patterns";
    }
    
    // ========== CALIBRATION IMPLEMENTATION ==========
    
    public boolean runCalibration() {
        try {
            telemetry.addData("Info", "Starting Enhanced 3×4 Kinematic Matrix Calibration");
            telemetry.addData("Info", String.format("Measurement time: %.1f sec, Steady-state wait: %.1f sec", 
                _8_KinematicMatrix.MEASUREMENT_TIME, _8_KinematicMatrix.STEADY_STATE_WAIT_TIME));
            telemetry.update();
            
            calibrationData.clear();
            
            // Define comprehensive motion patterns using consistent calibration velocities
            double vLin = calibrationLinearVelocity;  // ~23.4 in/s (70% of max)
            double vAng = calibrationAngularVelocity;  // ~70% of max angular velocity
            double vDiag = vLin * 0.707;  // Diagonal velocity (vx² + vy² = vLin²)
            double vComb = vLin * 0.8;    // Combined motion velocity (slightly reduced)
            double aComb = vAng * 0.6;    // Combined angular velocity (reduced for stability)
            
            MotionPattern[] patterns = {
                // Pure motions (baseline) - CRITICAL for basic calibration
                new MotionPattern("Pure Forward", vLin, 0.0, 0.0),
                new MotionPattern("Pure Backward", -vLin, 0.0, 0.0),
                new MotionPattern("Pure Left", 0.0, vLin, 0.0),
                new MotionPattern("Pure Right", 0.0, -vLin, 0.0),
                new MotionPattern("Pure Rotation CW", 0.0, 0.0, vAng),
                new MotionPattern("Pure Rotation CCW", 0.0, 0.0, -vAng),
                
                // Diagonal motions - test cross-wheel coupling
                new MotionPattern("NE Diagonal", vDiag, vDiag, 0.0),
                new MotionPattern("NW Diagonal", -vDiag, vDiag, 0.0),
                new MotionPattern("SW Diagonal", -vDiag, -vDiag, 0.0),
                new MotionPattern("SE Diagonal", vDiag, -vDiag, 0.0),
                
                // Combined translation + rotation (CRITICAL for cross-coupling)
                new MotionPattern("Forward + Rotation", vComb, 0.0, aComb),
                new MotionPattern("Left + Rotation", 0.0, vComb, aComb),
                new MotionPattern("Diagonal + Rotation", vComb*0.707, vComb*0.707, aComb),
                new MotionPattern("Forward + Counter-Rotation", vComb, 0.0, -aComb),
                new MotionPattern("Left + Counter-Rotation", 0.0, vComb, -aComb),
                
                // Additional patterns for robustness
                new MotionPattern("Slow Combined", vComb*0.6, vComb*0.6, aComb*0.5),
                new MotionPattern("Fast Combined", vComb, 0.0, aComb*0.8),
                new MotionPattern("Complex Motion", vComb*0.8, -vComb*0.6, aComb*0.7)
            };
            
            // Execute all patterns
            for (int i = 0; i < patterns.length; i++) {
                MotionPattern pattern = patterns[i];
                telemetry.addData("Info", String.format("Pattern %d/%d: %s", i+1, patterns.length, pattern.name));
                telemetry.update();
                
                if (!executeMotionPattern(pattern.vx, pattern.vy, pattern.omega, pattern.name)) {
                    telemetry.addData("ERROR", "Failed to execute pattern: " + pattern.name);
                    telemetry.update();
                    return false;
                }
                
                // Brief pause between patterns
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            
            telemetry.addData("Info", String.format("Data collection complete: %d samples", calibrationData.size()));
            telemetry.update();
            
            // Compute calibration matrix
            if (!computeCalibrationMatrix()) {
                telemetry.addData("ERROR", "Failed to compute calibration matrix");
                telemetry.update();
                return false;
            }
            
            // Validate calibration quality
            if (!validateCalibration()) {
                telemetry.addData("WARNING", "Calibration quality may be insufficient");
                telemetry.update();
            }
            
            telemetry.addData("Info", "Enhanced 3×4 Kinematic Matrix Calibration completed successfully");
            telemetry.update();
            return true;
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Calibration failed: " + e.getMessage());
            telemetry.update();
            return false;
        }
    }
    
    /**
     * Execute single motion pattern with time-based measurement
     */
    private boolean executeMotionPattern(double vx_cmd, double vy_cmd, double omega_cmd, String patternName) {
        try {
            telemetry.addData("Info", String.format("Executing %s: vx=%.1f, vy=%.1f, ω=%.1f", 
                patternName, vx_cmd, vy_cmd, omega_cmd));
            telemetry.update();
            
            // Reset odometry to clean starting point
            odometryManager.resetPosition(0, 0);
            
            // Start motion using DriveHardware directly (with 8-factor scaling)
            motionExecutor.getDriveHardware().setRobotVelocity(vx_cmd, vy_cmd, omega_cmd, MotionState.CoordinateMode.ROBOT_CENTRIC, 0.0);
            
            // Wait for steady state (0.5 seconds) with timeout protection
            ElapsedTime steadyStateTimer = new ElapsedTime();
            double maxSteadyStateTime = _8_KinematicMatrix.STEADY_STATE_WAIT_TIME + 2.0; // Add 2 second safety margin
            while (steadyStateTimer.seconds() < _8_KinematicMatrix.STEADY_STATE_WAIT_TIME) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return false; // Exit gracefully on interruption
                }
                
                // FIX BUG #17: Add timeout protection to prevent infinite hanging
                if (steadyStateTimer.seconds() > maxSteadyStateTime) {
                    telemetry.addData("ERROR", "Steady state timeout exceeded - aborting pattern");
                    telemetry.update();
                    motionExecutor.stop();
                    return false;
                }
            }
            
            // Record initial position/heading
            double x_initial = odometryManager.getX();
            double y_initial = odometryManager.getY();
            double heading_initial = odometryManager.getHeading();
            
            // Execute motion for exactly MEASUREMENT_TIME seconds with timeout protection
            ElapsedTime measurementTimer = new ElapsedTime();
            double maxMeasurementTime = _8_KinematicMatrix.MEASUREMENT_TIME + 5.0; // Add 5 second safety margin
            while (measurementTimer.seconds() < _8_KinematicMatrix.MEASUREMENT_TIME) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    motionExecutor.stop();
                    return false; // Exit gracefully on interruption
                }
                
                // FIX BUG #17: Add timeout protection to prevent infinite hanging
                if (measurementTimer.seconds() > maxMeasurementTime) {
                    telemetry.addData("ERROR", "Measurement timeout exceeded - aborting pattern");
                    telemetry.update();
                    motionExecutor.stop();
                    return false;
                }
                
                // Optional: Verify motion is still executing
                if (!motionExecutor.isExecuting()) {
                    telemetry.addData("WARNING", "Motion stopped unexpectedly during measurement");
                    telemetry.update();
                }
            }
            
            // Record final position/heading
            double x_final = odometryManager.getX();
            double y_final = odometryManager.getY();
            double heading_final = odometryManager.getHeading();
            
            // Stop motion
            motionExecutor.stop();
            
            // Calculate actual velocities from displacement and time
            double vx_actual = (x_final - x_initial) / _8_KinematicMatrix.MEASUREMENT_TIME;
            double vy_actual = (y_final - y_initial) / _8_KinematicMatrix.MEASUREMENT_TIME;
            double omega_actual = normalizeAngle(heading_final - heading_initial) / _8_KinematicMatrix.MEASUREMENT_TIME;
            
            // Get commanded wheel velocities (from ideal kinematics)
            MecanumKinematics.WheelVelocities commandedWheels = 
                MecanumKinematics.robotVelocitiesToWheelVelocities(vx_cmd, vy_cmd, omega_cmd);
            
            // Store calibration data point
            calibrationData.add(new CalibrationDataPoint(
                vx_cmd, vy_cmd, omega_cmd,
                commandedWheels,
                vx_actual, vy_actual, omega_actual,
                _8_KinematicMatrix.MEASUREMENT_TIME, _8_KinematicMatrix.STEADY_STATE_WAIT_TIME
            ));
            
            // Log results
            telemetry.addData("Info", String.format("Commanded: vx=%.2f, vy=%.2f, ω=%.2f", vx_cmd, vy_cmd, omega_cmd));
            telemetry.addData("Info", String.format("Actual:    vx=%.2f, vy=%.2f, ω=%.2f", vx_actual, vy_actual, omega_actual));
            telemetry.addData("Info", String.format("Error:     vx=%.2f, vy=%.2f, ω=%.2f", 
                vx_actual - vx_cmd, vy_actual - vy_cmd, omega_actual - omega_cmd));
            telemetry.update();
            
            return true;
            
        } catch (Exception e) {
            motionExecutor.stop();
            telemetry.addData("ERROR", "Motion pattern execution failed: " + e.getMessage());
            telemetry.update();
            return false;
        }
    }
    
    /**
     * Compute 3×4 calibration matrix using enhanced least squares
     */
    private boolean computeCalibrationMatrix() {
        try {
            int N = calibrationData.size();
            if (N < 12) {  // Need at least 12 data points for 12 unknowns (each point gives 3 equations)
                telemetry.addData("ERROR", "Insufficient data points: " + N + " (need at least 12)");
                telemetry.update();
                return false;
            }
            
            telemetry.addData("Info", String.format("Computing matrix from %d data points", N));
            telemetry.update();
            
            // Set up least squares system: Y = A × K
            double[][] A = new double[3*N][12];  // Measurement matrix
            double[] Y = new double[3*N];        // Measured velocities
            
            // Fill measurement matrix with commanded wheel velocities
            for (int i = 0; i < N; i++) {
                CalibrationDataPoint point = calibrationData.get(i);
                
                // vx equation: row 3*i
                A[3*i][0] = point.wheelVelocities.frontLeft;   // K11
                A[3*i][1] = point.wheelVelocities.frontRight;  // K12
                A[3*i][2] = point.wheelVelocities.backLeft;    // K13
                A[3*i][3] = point.wheelVelocities.backRight;   // K14
                for (int j = 4; j < 12; j++) A[3*i][j] = 0;
                Y[3*i] = point.vx_actual;  // ACTUAL measured velocity
                
                // vy equation: row 3*i+1
                for (int j = 0; j < 4; j++) A[3*i+1][j] = 0;
                A[3*i+1][4] = point.wheelVelocities.frontLeft;   // K21
                A[3*i+1][5] = point.wheelVelocities.frontRight;  // K22
                A[3*i+1][6] = point.wheelVelocities.backLeft;    // K23
                A[3*i+1][7] = point.wheelVelocities.backRight;   // K24
                for (int j = 8; j < 12; j++) A[3*i+1][j] = 0;
                Y[3*i+1] = point.vy_actual;  // ACTUAL measured velocity
                
                // ω equation: row 3*i+2
                for (int j = 0; j < 8; j++) A[3*i+2][j] = 0;
                A[3*i+2][8]  = point.wheelVelocities.frontLeft;   // K31
                A[3*i+2][9]  = point.wheelVelocities.frontRight;  // K32
                A[3*i+2][10] = point.wheelVelocities.backLeft;    // K33
                A[3*i+2][11] = point.wheelVelocities.backRight;   // K34
                Y[3*i+2] = point.omega_actual;  // ACTUAL measured angular velocity
            }
            
            // Solve least squares using robust SVD decomposition
            SVDSolution svdResult = solveLeastSquaresSVD(A, Y);
            double[] coefficients = svdResult.coefficients;
            double conditionNumber = svdResult.conditionNumber;
            int matrixRank = svdResult.rank;
            
            // Reshape into 3×4 matrix
            double[][] K = new double[3][4];
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 4; col++) {
                    K[row][col] = coefficients[row * 4 + col];
                }
            }
            
            // Calculate comprehensive quality metrics
            double rSquared = calculateRSquared(A, Y, coefficients);
            double[] residuals = calculateResiduals(A, Y, coefficients);
            double rmsResidual = calculateRMSResidual(residuals);
            
            // Enhanced logging with SVD-specific information
            telemetry.addData("Info", String.format("Matrix rank: %d/%d (full rank = %s)", 
                matrixRank, Math.min(A.length, A[0].length), 
                (matrixRank == Math.min(A.length, A[0].length)) ? "YES" : "NO"));
            telemetry.addData("Info", String.format("Singular values: [%.2e, %.2e, %.2e, %.2e]", 
                svdResult.singularValues[0], svdResult.singularValues[1], 
                svdResult.singularValues[2], svdResult.singularValues[3]));
            telemetry.update();
            
            // Store calibration results
            CalibrationCoefficients.setKinematicMatrix(K, rSquared, conditionNumber, rmsResidual, N);
            
            // Log detailed results
            telemetry.addData("Info", "=== CALIBRATION MATRIX RESULTS ===");
            telemetry.addData("Info", "vx coefficients: [" + String.format("%.4f, %.4f, %.4f, %.4f", K[0][0], K[0][1], K[0][2], K[0][3]) + "]");
            telemetry.addData("Info", "vy coefficients: [" + String.format("%.4f, %.4f, %.4f, %.4f", K[1][0], K[1][1], K[1][2], K[1][3]) + "]");
            telemetry.addData("Info", "ω  coefficients: [" + String.format("%.4f, %.4f, %.4f, %.4f", K[2][0], K[2][1], K[2][2], K[2][3]) + "]");
            telemetry.addData("Info", String.format("Model R² = %.4f", rSquared));
            telemetry.addData("Info", String.format("Condition Number = %.2f", conditionNumber));
            telemetry.addData("Info", String.format("RMS Residual = %.4f", rmsResidual));
            telemetry.update();
            
            return true;
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Matrix computation failed: " + e.getMessage());
            telemetry.update();
            return false;
        }
    }
    
    /**
     * Validate calibration quality
     */
    private boolean validateCalibration() {
        double rSquared = CalibrationCoefficients.KINEMATIC_CALIBRATION_R_SQUARED;
        double conditionNumber = CalibrationCoefficients.KINEMATIC_CALIBRATION_CONDITION_NUMBER;
        double rmsResidual = CalibrationCoefficients.KINEMATIC_CALIBRATION_RMS_RESIDUAL;
        
        boolean isGood = true;
        
        if (rSquared < _8_KinematicMatrix.MIN_R_SQUARED) {
            telemetry.addData("WARNING", String.format("Low R² = %.4f (should be > %.2f)", rSquared, _8_KinematicMatrix.MIN_R_SQUARED)); telemetry.update();
            isGood = false;
        }
        
        if (conditionNumber > _8_KinematicMatrix.MAX_CONDITION_NUMBER) {
            telemetry.addData("WARNING", String.format("High condition number = %.2f (should be < %.1f)", conditionNumber, _8_KinematicMatrix.MAX_CONDITION_NUMBER)); telemetry.update();
            isGood = false;
        }
        
        if (rmsResidual > _8_KinematicMatrix.MAX_RMS_RESIDUAL) {
            telemetry.addData("WARNING", String.format("High RMS residual = %.4f (should be < %.1f)", rmsResidual, _8_KinematicMatrix.MAX_RMS_RESIDUAL)); telemetry.update();
            isGood = false;
        }
        
        if (isGood) {
            telemetry.addData("Info", "✅ Calibration quality is excellent"); telemetry.update();
        } else {
            telemetry.addData("WARNING", "⚠️ Calibration quality may be insufficient - consider recalibrating"); telemetry.update();
        }
        
        return isGood;
    }
    
    /**
     * Solve least squares using robust SVD decomposition
     * 
     * This replaces the previous Gaussian elimination approach with SVD-based
     * pseudo-inverse calculation, which is numerically stable and handles
     * rank-deficient matrices gracefully.
     * 
     * FIXES BUGS #13, #14, #15, #16: Eliminates all division by zero issues
     * and provides accurate condition number calculation.
     */
    private SVDSolution solveLeastSquaresSVD(double[][] A, double[] Y) {
        int m = A.length;    // Number of equations
        int n = A[0].length; // Number of unknowns (12)
        
        try {
            // Convert to EJML matrices
            DMatrixRMaj matrixA = new DMatrixRMaj(m, n);
            DMatrixRMaj vectorY = new DMatrixRMaj(m, 1);
            
            // Fill matrices with data
            for (int i = 0; i < m; i++) {
                for (int j = 0; j < n; j++) {
                    matrixA.set(i, j, A[i][j]);
                }
                vectorY.set(i, 0, Y[i]);
            }
            
            // Perform SVD: A = U * S * V^T
            SingularValueDecomposition_F64<DMatrixRMaj> svd = 
                DecompositionFactory_DDRM.svd(m, n, true, true, false);
            
            if (!svd.decompose(matrixA)) {
                throw new RuntimeException("SVD decomposition failed - matrix may be invalid");
            }
            
            // Get SVD components
            DMatrixRMaj U = svd.getU(null, false);
            DMatrixRMaj S = svd.getW(null);
            DMatrixRMaj V = svd.getV(null, false);
            
            // Calculate condition number (ratio of largest to smallest singular value)
            double[] singularValues = new double[Math.min(m, n)];
            for (int i = 0; i < singularValues.length; i++) {
                singularValues[i] = S.get(i, i);
            }
            
            double maxSV = 0.0;
            double minSV = Double.MAX_VALUE;
            int rank = 0;
            double tolerance = 1e-12; // Tolerance for rank determination
            
            for (double sv : singularValues) {
                if (sv > tolerance) {
                    maxSV = Math.max(maxSV, sv);
                    minSV = Math.min(minSV, sv);
                    rank++;
                }
            }
            
            double conditionNumber = (minSV > tolerance) ? (maxSV / minSV) : Double.POSITIVE_INFINITY;
            
            // Solve using pseudo-inverse with automatic regularization
            DMatrixRMaj solution = new DMatrixRMaj(n, 1);
            
            // Use EJML's built-in solver which handles rank deficiency automatically
            if (!CommonOps_DDRM.solve(matrixA, vectorY, solution)) {
                // If direct solve fails, use pseudo-inverse approach
                DMatrixRMaj pinv = new DMatrixRMaj(n, m);
                CommonOps_DDRM.pinv(matrixA, pinv);
                CommonOps_DDRM.mult(pinv, vectorY, solution);
            }
            
            // Convert solution back to array
            double[] result = new double[n];
            for (int i = 0; i < n; i++) {
                result[i] = solution.get(i, 0);
                
                // Validate solution values (fixes BUG #14)
                if (!Double.isFinite(result[i])) {
                    throw new RuntimeException("SVD solution contains invalid values (NaN/Infinity)");
                }
            }
            
            return new SVDSolution(result, conditionNumber, rank, singularValues);
            
        } catch (Exception e) {
            throw new RuntimeException("SVD-based least squares solution failed: " + e.getMessage(), e);
        }
    }
    
    /**
     * Container for SVD solution results
     */
    private static class SVDSolution {
        public final double[] coefficients;
        public final double conditionNumber;
        public final int rank;
        public final double[] singularValues;
        
        public SVDSolution(double[] coefficients, double conditionNumber, int rank, double[] singularValues) {
            this.coefficients = coefficients;
            this.conditionNumber = conditionNumber;
            this.rank = rank;
            this.singularValues = singularValues;
        }
    }
    
    // NOTE: Old Gaussian elimination method removed and replaced with robust SVD approach
    // This eliminates BUGS #13, #14 (division by zero and NaN propagation)
    
    /**
     * Calculate R-squared (coefficient of determination) with robust handling
     * 
     * FIXES BUG #15: Handles division by zero when all Y values are identical
     */
    private double calculateRSquared(double[][] A, double[] Y, double[] coefficients) {
        // Calculate predicted values
        double[] predicted = new double[Y.length];
        for (int i = 0; i < Y.length; i++) {
            predicted[i] = 0;
            for (int j = 0; j < coefficients.length; j++) {
                predicted[i] += A[i][j] * coefficients[j];
            }
        }
        
        // Calculate mean of observed values
        double meanY = 0;
        for (double y : Y) {
            meanY += y;
        }
        meanY /= Y.length;
        
        // Calculate sum of squares
        double ssRes = 0; // Sum of squares of residuals
        double ssTot = 0; // Total sum of squares
        
        for (int i = 0; i < Y.length; i++) {
            ssRes += Math.pow(Y[i] - predicted[i], 2);
            ssTot += Math.pow(Y[i] - meanY, 2);
        }
        
        // FIX BUG #15: Handle case where all Y values are identical (ssTot = 0)
        if (ssTot < 1e-12) {
            // If all observed values are the same, R² is undefined
            // Return 1.0 if predictions are also constant and match observations
            // Return 0.0 if predictions vary when observations don't
            return (ssRes < 1e-12) ? 1.0 : 0.0;
        }
        
        double rSquared = 1 - (ssRes / ssTot);
        
        // Ensure R² is in valid range [0, 1] (can be negative for very poor fits)
        return Math.max(0.0, Math.min(1.0, rSquared));
    }
    
    // NOTE: Old condition number calculation removed and replaced with accurate SVD-based calculation
    // This eliminates BUG #16 (edge cases in diagonal dominance estimation)
    // The condition number is now calculated directly from SVD singular values in solveLeastSquaresSVD()
    
    /**
     * Calculate residuals
     */
    private double[] calculateResiduals(double[][] A, double[] Y, double[] coefficients) {
        double[] residuals = new double[Y.length];
        for (int i = 0; i < Y.length; i++) {
            double predicted = 0;
            for (int j = 0; j < coefficients.length; j++) {
                predicted += A[i][j] * coefficients[j];
            }
            residuals[i] = Y[i] - predicted;
        }
        return residuals;
    }
    
    /**
     * Calculate RMS residual
     */
    private double calculateRMSResidual(double[] residuals) {
        double sumSquares = 0;
        for (double residual : residuals) {
            sumSquares += residual * residual;
        }
        return Math.sqrt(sumSquares / residuals.length);
    }
    
    /**
     * Normalize angle to [-180, 180] range
     */
    protected double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    
    /**
     * Data point for calibration
     */
    private static class CalibrationDataPoint {
        // Commanded velocities
        public double vx_commanded;
        public double vy_commanded;
        public double omega_commanded;
        
        // Commanded wheel velocities (from ideal kinematics)
        public MecanumKinematics.WheelVelocities wheelVelocities;
        
        // Measured actual velocities (from odometry + time)
        public double vx_actual;
        public double vy_actual;
        public double omega_actual;
        
        // Measurement metadata
        public double measurementTime;
        public double steadyStateTime;
        
        public CalibrationDataPoint(double vx_cmd, double vy_cmd, double omega_cmd,
                                   MecanumKinematics.WheelVelocities wheels,
                                   double vx_act, double vy_act, double omega_act,
                                   double measTime, double steadyTime) {
            this.vx_commanded = vx_cmd;
            this.vy_commanded = vy_cmd;
            this.omega_commanded = omega_cmd;
            this.wheelVelocities = wheels;
            this.vx_actual = vx_act;
            this.vy_actual = vy_act;
            this.omega_actual = omega_act;
            this.measurementTime = measTime;
            this.steadyStateTime = steadyTime;
        }
    }
    
    /**
     * Motion pattern definition
     */
    private static class MotionPattern {
        public String name;
        public double vx, vy, omega;
        
        public MotionPattern(String name, double vx, double vy, double omega) {
            this.name = name;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }
}

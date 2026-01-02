package org.firstinspires.ftc.teamcode.utils;

import android.content.Context;
import android.os.Environment;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Detailed logging utility for state machine debugging.
 * 
 * Logs to both telemetry (if enabled) and a file for detailed analysis.
 * 
 * Log format includes:
 * - Timestamp
 * - Component name (GameManager, DriverManager, etc.)
 * - State transitions (previous -> current -> next)
 * - Settings/expectations (targets, timeouts, etc.)
 * - Current status (position, velocity, state, etc.)
 */
public class StateMachineLogger {
    
    private static final String LOG_DIR = "StateMachineLogs";
    private static final SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US);
    
    /**
     * Helper to repeat a string (for compatibility with older Android versions).
     */
    private static String repeat(String str, int count) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < count; i++) {
            sb.append(str);
        }
        return sb.toString();
    }
    
    private final String componentName;
    private final boolean enabled;
    private final Telemetry telemetry;
    private final File baseLogDir;  // Base directory for log files (app-scoped)
    private FileWriter logWriter;
    private String logFilePath;
    private long startTimeMs;
    
    // Time-based throttling: only log if enough time has passed (or if forced)
    private long lastLogTimeMs = 0;
    private static final long MIN_LOG_INTERVAL_MS = 100;  // 10 Hz max logging rate
    
    // Buffering for performance: use StringBuilder for efficient string building
    private final StringBuilder logBuffer = new StringBuilder();
    private int bufferLines = 0;
    private static final int MAX_BUFFER_LINES = 10;  // Flush after N lines (reduced for more frequent flushes)
    private static final long BUFFER_FLUSH_INTERVAL_MS = 500;  // Flush every 500ms (reduced for long runs)
    private long lastFlushTimeMs = 0;
    
    // Telemetry control: optional, shows short summary lines
    private boolean telemetryEnabled = false;  // Disabled by default to avoid spam
    
    // Logging categories
    public enum LogCategory {
        STATE_TRANSITION,    // State changes
        SETTINGS,           // Configuration/expectations
        STATUS,             // Current robot status
        ERROR,              // Errors/warnings
        DECISION            // Decision points (if/else branches)
    }
    
    /**
     * Create a logger for a specific component.
     * Uses default storage location (app-scoped external files directory if available).
     * 
     * @param componentName Name of the component (e.g., "GameManager", "DriverManager")
     * @param enabled Whether logging is enabled (default true for detailed debugging)
     * @param telemetry Telemetry instance (can be null if not using telemetry)
     */
    public StateMachineLogger(String componentName, boolean enabled, Telemetry telemetry) {
        this.componentName = componentName;
        this.enabled = enabled;
        this.telemetry = telemetry;
        this.baseLogDir = getDefaultLogDirectory();
        this.startTimeMs = System.currentTimeMillis();
        this.lastLogTimeMs = startTimeMs;
        this.lastFlushTimeMs = startTimeMs;
        
        if (enabled) {
            initializeLogFile();
        }
    }
    
    /**
     * Create a logger for a specific component with a custom base directory.
     * 
     * @param componentName Name of the component (e.g., "GameManager", "DriverManager")
     * @param enabled Whether logging is enabled (default true for detailed debugging)
     * @param telemetry Telemetry instance (can be null if not using telemetry)
     * @param baseLogDir Base directory for log files (null to use default app-scoped storage)
     */
    public StateMachineLogger(String componentName, boolean enabled, Telemetry telemetry, File baseLogDir) {
        this.componentName = componentName;
        this.enabled = enabled;
        this.telemetry = telemetry;
        this.baseLogDir = baseLogDir != null ? baseLogDir : getDefaultLogDirectory();
        this.startTimeMs = System.currentTimeMillis();
        this.lastLogTimeMs = startTimeMs;
        this.lastFlushTimeMs = startTimeMs;
        
        if (enabled) {
            initializeLogFile();
        }
    }
    
    /**
     * Create a logger using Context to get app-scoped external files directory.
     * This is the recommended approach for OpMode contexts.
     * 
     * @param componentName Name of the component (e.g., "GameManager", "DriverManager")
     * @param enabled Whether logging is enabled (default true for detailed debugging)
     * @param telemetry Telemetry instance (can be null if not using telemetry)
     * @param context Android Context (e.g., from hardwareMap.appContext)
     */
    public StateMachineLogger(String componentName, boolean enabled, Telemetry telemetry, Context context) {
        this.componentName = componentName;
        this.enabled = enabled;
        this.telemetry = telemetry;
        this.baseLogDir = context != null ? getAppScopedLogDirectory(context) : getDefaultLogDirectory();
        this.startTimeMs = System.currentTimeMillis();
        this.lastLogTimeMs = startTimeMs;
        this.lastFlushTimeMs = startTimeMs;
        
        if (enabled) {
            initializeLogFile();
        }
    }
    
    /**
     * Get app-scoped external files directory for logging (recommended for OpModes).
     * This avoids scoped storage issues and doesn't require extra permissions.
     * 
     * @param context Android Context
     * @return File directory for logs, or null if unavailable
     */
    private static File getAppScopedLogDirectory(Context context) {
        try {
            File externalFilesDir = context.getExternalFilesDir(null);
            if (externalFilesDir != null) {
                return new File(externalFilesDir, LOG_DIR);
            }
        } catch (Exception e) {
            RobotLog.w("StateMachineLogger: Failed to get app-scoped directory: %s", e.getMessage());
        }
        return null;
    }
    
    /**
     * Get default log directory with fallback chain:
     * 1. AppUtil.getSettingsFile() (FTC SDK standard)
     * 2. Environment.getExternalStorageDirectory() (legacy, with warning)
     * 
     * @return File directory for logs
     */
    private static File getDefaultLogDirectory() {
        // Try AppUtil first (FTC SDK standard)
        try {
            File settingsFile = AppUtil.getInstance().getSettingsFile("test");
            if (settingsFile != null) {
                File parentDir = settingsFile.getParentFile();
                if (parentDir != null) {
                    return new File(parentDir, LOG_DIR);
                }
            }
        } catch (Exception e) {
            RobotLog.w("StateMachineLogger: AppUtil.getSettingsFile() failed: %s", e.getMessage());
        }
        
        // Fallback to legacy external storage (with warning)
        try {
            File externalStorage = Environment.getExternalStorageDirectory();
            if (externalStorage != null) {
                RobotLog.w("StateMachineLogger: Using legacy external storage (may fail on modern Android): %s", 
                        externalStorage.getAbsolutePath());
                return new File(externalStorage, LOG_DIR);
            }
        } catch (Exception e) {
            RobotLog.e("StateMachineLogger: Failed to get external storage directory: %s", e.getMessage());
        }
        
        // Last resort: return null (will log error during initialization)
        return null;
    }
    
    /**
     * Enable or disable telemetry output (disabled by default to avoid spam).
     * 
     * @param enabled Whether to show short summary lines in telemetry
     */
    public void setTelemetryEnabled(boolean enabled) {
        this.telemetryEnabled = enabled;
    }
    
    /**
     * Initialize log file with timestamp.
     * Uses app-scoped external files directory (recommended) or falls back to legacy storage.
     */
    private void initializeLogFile() {
        if (baseLogDir == null) {
            RobotLog.e("StateMachineLogger: No valid log directory available - file logging disabled");
            logFilePath = null;
            if (telemetry != null) {
                telemetry.addData("Logger Error", String.format("%s: No log directory", componentName));
            }
            return;
        }
        
        try {
            // Create log directory if it doesn't exist
            if (!baseLogDir.exists()) {
                boolean created = baseLogDir.mkdirs();
                if (!created) {
                    RobotLog.e("StateMachineLogger: Could not create log directory: %s", baseLogDir.getAbsolutePath());
                    logFilePath = null;
                    if (telemetry != null) {
                        telemetry.addData("Logger Error", String.format("%s: Cannot create directory", componentName));
                    }
                    return;
                }
            }
            
            // Create log file with timestamp
            String timestamp = DATE_FORMAT.format(new Date());
            String filename = String.format("%s_%s.log", componentName, timestamp);
            File logFile = new File(baseLogDir, filename);
            logFilePath = logFile.getAbsolutePath();
            
            logWriter = new FileWriter(logFile, true);
            logHeader();
            
            RobotLog.i("StateMachineLogger: Logging to %s", logFilePath);
            
            if (telemetry != null) {
                telemetry.addData("Logger", String.format("%s: %s", componentName, filename));
            }
        } catch (IOException e) {
            RobotLog.e("StateMachineLogger: Failed to create log file: %s", e.getMessage());
            logFilePath = null;  // Mark as failed
            if (telemetry != null) {
                telemetry.addData("Logger Error", String.format("%s: %s", componentName, e.getMessage()));
            }
        } catch (Exception e) {
            // Catch any other exceptions (e.g., permission issues)
            RobotLog.e("StateMachineLogger: Unexpected error creating log file: %s", e.getMessage());
            logFilePath = null;  // Mark as failed
            if (telemetry != null) {
                telemetry.addData("Logger Error", String.format("%s: %s", componentName, e.getMessage()));
            }
        }
    }
    
    /**
     * Write log file header with initialization info.
     */
    private void logHeader() throws IOException {
        if (logWriter == null) return;
        
        logWriter.write(repeat("=", 80) + "\n");
        logWriter.write(String.format("State Machine Log: %s\n", componentName));
        logWriter.write(String.format("Started: %s\n", new Date()));
        logWriter.write(repeat("=", 80) + "\n\n");
        logWriter.flush();
    }
    
    /**
     * Log a state transition.
     * 
     * @param previousState Previous state (can be null)
     * @param currentState Current state
     * @param nextState Next expected state (can be null)
     * @param reason Reason for transition (can be null)
     */
    public void logStateTransition(Object previousState, Object currentState, Object nextState, String reason) {
        if (!enabled) return;
        
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("[STATE_TRANSITION] t=%.3fs\n", getElapsedTimeSec()));
        sb.append(String.format("  Previous: %s\n", previousState != null ? previousState.toString() : "null"));
        sb.append(String.format("  Current:  %s\n", currentState != null ? currentState.toString() : "null"));
        sb.append(String.format("  Next:     %s\n", nextState != null ? nextState.toString() : "null"));
        if (reason != null) {
            sb.append(String.format("  Reason:   %s\n", reason));
        }
        sb.append("\n");
        
        // State transitions are important - always log (force=true)
        writeLog(sb.toString(), true);
        
        // Show short summary in telemetry
        if (telemetryEnabled && telemetry != null) {
            String summary = String.format("%s: %s -> %s", componentName,
                    previousState != null ? previousState.toString() : "null",
                    currentState != null ? currentState.toString() : "null");
            writeTelemetryLine(summary);
        }
    }
    
    /**
     * Log settings/expectations at a point in time.
     * 
     * @param settings Map-like string of settings (e.g., "targetX=24.5, timeout=3.0")
     */
    public void logSettings(String settings) {
        if (!enabled) return;
        
        String log = String.format("[SETTINGS] t=%.3fs | %s\n\n", getElapsedTimeSec(), settings);
        writeLog(log, false);  // Settings are less critical - can be throttled
    }
    
    /**
     * Log current robot status/facts.
     * 
     * @param status Map-like string of status (e.g., "x=24.5, y=12.3, heading=45.0, velocity=30.0")
     */
    public void logStatus(String status) {
        if (!enabled) return;
        
        String log = String.format("[STATUS] t=%.3fs | %s\n", getElapsedTimeSec(), status);
        writeLog(log, false);  // Status updates are frequent - throttle them
    }
    
    /**
     * Log a decision point.
     * 
     * @param condition Condition being evaluated
     * @param result Result of condition (true/false)
     * @param action Action taken based on result
     */
    public void logDecision(String condition, boolean result, String action) {
        if (!enabled) return;
        
        String log = String.format("[DECISION] t=%.3fs | Condition: %s = %s | Action: %s\n\n",
                getElapsedTimeSec(), condition, result, action);
        writeLog(log, false);  // Decisions can be throttled
    }
    
    /**
     * Log an error or warning.
     * 
     * @param level "ERROR" or "WARNING"
     * @param message Error message
     */
    public void logError(String level, String message) {
        if (!enabled) return;
        
        String log = String.format("[%s] t=%.3fs | %s\n\n", level, getElapsedTimeSec(), message);
        writeLog(log, true);  // Errors are critical - always log (force=true)
        
        // Show error in telemetry
        if (telemetryEnabled && telemetry != null) {
            String summary = String.format("%s %s: %s", componentName, level, message);
            writeTelemetryLine(summary);
        }
    }
    
    /**
     * Log a detailed entry with all information.
     * 
     * @param previousState Previous state
     * @param currentState Current state
     * @param nextState Next expected state
     * @param settings Settings/expectations
     * @param status Current status/facts
     * @param reason Reason for transition (optional)
     */
    public void logDetailed(Object previousState, Object currentState, Object nextState,
                           String settings, String status, String reason) {
        if (!enabled) return;
        
        StringBuilder sb = new StringBuilder();
        sb.append(repeat("=", 80) + "\n");
        sb.append(String.format("[DETAILED] t=%.3fs\n", getElapsedTimeSec()));
        sb.append("--- STATE TRANSITION ---\n");
        sb.append(String.format("  Previous: %s\n", previousState != null ? previousState.toString() : "null"));
        sb.append(String.format("  Current:  %s\n", currentState != null ? currentState.toString() : "null"));
        sb.append(String.format("  Next:     %s\n", nextState != null ? nextState.toString() : "null"));
        if (reason != null) {
            sb.append(String.format("  Reason:   %s\n", reason));
        }
        sb.append("\n--- SETTINGS/EXPECTATIONS ---\n");
        sb.append(String.format("  %s\n", settings != null ? settings : "none"));
        sb.append("\n--- CURRENT STATUS/FACTS ---\n");
        sb.append(String.format("  %s\n", status != null ? status : "none"));
        sb.append(repeat("=", 80) + "\n\n");
        
        writeLog(sb.toString(), true);  // Detailed logs are important - always log (force=true)
    }
    
    /**
     * Write log entry to buffer (will be flushed periodically for performance).
     * Uses time-based throttling to avoid excessive logging.
     * 
     * @param log Log entry to write
     * @param force If true, always log regardless of throttling (for state transitions/errors)
     */
    private void writeLog(String log, boolean force) {
        // Time-based throttling: skip if not enough time has passed (unless forced)
        if (!force) {
            long now = System.currentTimeMillis();
            if (now - lastLogTimeMs < MIN_LOG_INTERVAL_MS) {
                return;  // Skip this log entry due to throttling
            }
            lastLogTimeMs = now;
        } else {
            // For forced entries, update time but don't skip
            lastLogTimeMs = System.currentTimeMillis();
        }
        
        // Add to buffer (non-blocking, thread-safe)
        synchronized (logBuffer) {
            logBuffer.append(log);
            bufferLines++;
        }
        
        // Write to RobotLog only for forced entries (state transitions/errors)
        // This reduces logcat spam while still capturing important events
        if (force) {
            RobotLog.dd("StateMachineLogger", log);
        }
        
        // Flush buffer if threshold reached or if forced (non-blocking check)
        // Note: periodicFlush() should also be called in the update loop to ensure
        // logs are written even if buffer size threshold is never reached during long runs
        if (bufferLines >= MAX_BUFFER_LINES || force) {
            flushBuffer();
        }
    }
    
    /**
     * Write a short summary line to telemetry.
     * 
     * Note: This method calls telemetry.addLine(). The caller must call telemetry.update()
     * in their update loop for the message to appear on the Driver Station screen.
     * 
     * @param msg Short message to display
     */
    private void writeTelemetryLine(String msg) {
        if (telemetryEnabled && telemetry != null) {
            telemetry.addLine(msg);
            // Note: telemetry.update() must be called by the caller in their update loop
            // for this message to appear on the Driver Station screen.
        }
    }
    
    /**
     * Flush buffered log entries to disk.
     * Called periodically or when buffer threshold is reached.
     */
    private void flushBuffer() {
        if (logWriter == null) return;
        
        String toWrite;
        int linesToWrite;
        synchronized (logBuffer) {
            if (logBuffer.length() == 0) return;
            toWrite = logBuffer.toString();
            linesToWrite = bufferLines;
            logBuffer.setLength(0);  // Clear buffer
            bufferLines = 0;
        }
        
        try {
            logWriter.write(toWrite);
            logWriter.flush();
            lastFlushTimeMs = System.currentTimeMillis();
        } catch (IOException e) {
            RobotLog.e("StateMachineLogger: Failed to flush log buffer: %s", e.getMessage());
            // Put entries back in buffer (they'll be retried on next flush)
            synchronized (logBuffer) {
                logBuffer.insert(0, toWrite);
                bufferLines += linesToWrite;
            }
        }
    }
    
    /**
     * Periodic flush check (call from update loop if needed).
     * Flushes if enough time has passed since last flush.
     */
    public void periodicFlush() {
        long now = System.currentTimeMillis();
        if (now - lastFlushTimeMs >= BUFFER_FLUSH_INTERVAL_MS) {
            flushBuffer();
        }
    }
    
    /**
     * Force immediate flush of all buffered entries.
     * Useful before closing or when critical events occur.
     */
    public void forceFlush() {
        flushBuffer();
    }
    
    /**
     * Get elapsed time since logger creation in seconds.
     */
    private double getElapsedTimeSec() {
        return (System.currentTimeMillis() - startTimeMs) / 1000.0;
    }
    
    /**
     * Close log file (call when done).
     * Flushes any remaining buffered entries before closing.
     */
    public void close() {
        // Force flush any remaining buffered entries
        writeLog("\n" + repeat("=", 80) + "\n", true);
        writeLog(String.format("Log ended: %s\n", new Date()), true);
        writeLog(repeat("=", 80) + "\n", true);
        flushBuffer();  // Final flush
        
        if (logWriter != null) {
            try {
                logWriter.close();
                RobotLog.i("StateMachineLogger: Log file closed: %s", logFilePath);
            } catch (IOException e) {
                RobotLog.e("StateMachineLogger: Failed to close log file: %s", e.getMessage());
            } finally {
                logWriter = null;
            }
        }
    }
    
    /**
     * Get the log file path (for telemetry display).
     */
    public String getLogFilePath() {
        return logFilePath;
    }
}


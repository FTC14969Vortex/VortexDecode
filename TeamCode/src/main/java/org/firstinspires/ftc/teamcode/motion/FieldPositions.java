package org.firstinspires.ftc.teamcode.motion;

import java.util.HashMap;
import java.util.Map;
import java.util.Arrays;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/**
 * Manages named field positions for autonomous navigation.
 * 
 * This class provides a centralized way to define and access field positions
 * using meaningful names instead of raw coordinates. Positions can be:
 * - Predefined as constants (HIGH_BASKET, START_POSITION, etc.)
 * - Loaded from a properties file
 * - Added dynamically during runtime
 * 
 * All positions are stored in reference point coordinates, meaning they represent
 * where the robot's active reference point (configured in MotionConfig) should be
 * positioned on the field.
 * 
 * Example usage:
 * ```java
 * // Use predefined positions
 * motionExecutor.moveToPose(FieldPositions.HIGH_BASKET, 30.0);
 * 
 * // Load custom positions from file
 * FieldPositions.loadFromFile("/sdcard/FIRST/field_positions.properties");
 * motionExecutor.moveToPose(FieldPositions.get("custom_scoring_pos"), 25.0);
 * 
 * // Create path from multiple positions
 * FieldPose[] path = FieldPositions.createPath("start", "waypoint1", "scoring");
 * ```
 */
public class FieldPositions {
    
    // ========== STORAGE ==========
    
    /** Map of named positions */
    private static final Map<String, FieldPose> positions = new HashMap<>();
    
    // ========== DECODE 2025-2026 FIELD POSITIONS ==========
    // Field coordinate system: Origin at center (0,0), +Y toward Blue Alliance, +X away from audience
    // Red Alliance Area on LEFT from audience view (inverted configuration)
    
    // === RED ALLIANCE POSITIONS (Y < 0, toward Red Wall) ===
    
    // Start positions
    public static final FieldPose RED_START_LEFT = new FieldPose(-48.0, -60.0, 90.0);
    public static final FieldPose RED_START_RIGHT = new FieldPose(48.0, -60.0, 90.0);
    
    // Artifact intake positions (3 rows of coral)
    public static final FieldPose RED_INTAKE_1_START = new FieldPose(-36.0, -48.0, 90.0);
    public static final FieldPose RED_INTAKE_1_FINISH = new FieldPose(-36.0, -36.0, 90.0);
    public static final FieldPose RED_INTAKE_2_START = new FieldPose(0.0, -36.0, 90.0);
    public static final FieldPose RED_INTAKE_2_FINISH = new FieldPose(0.0, -24.0, 90.0);
    public static final FieldPose RED_INTAKE_3_START = new FieldPose(36.0, -24.0, 90.0);
    public static final FieldPose RED_INTAKE_3_FINISH = new FieldPose(36.0, -12.0, 90.0);
    
    // Shooting positions (targeting Blue GOAL)
    public static final FieldPose RED_SHOOTING_CLOSE = new FieldPose(0.0, -48.0, 0.0);
    public static final FieldPose RED_SHOOTING_MID = new FieldPose(0.0, -36.0, 0.0);
    public static final FieldPose RED_SHOOTING_LONG = new FieldPose(0.0, -24.0, 0.0);
    
    // Utility positions
    public static final FieldPose RED_BASE_ZONE = new FieldPose(-60.0, -60.0, 45.0);
    public static final FieldPose RED_LOADING_ZONE = new FieldPose(-60.0, -36.0, 0.0);
    public static final FieldPose RED_SECRET_TUNNEL = new FieldPose(0.0, -72.0, 90.0);
    public static final FieldPose RED_GATE_ZONE = new FieldPose(12.0, -72.0, 90.0);
    public static final FieldPose RED_PARKING = new FieldPose(-48.0, -48.0, 45.0);
    
    // === BLUE ALLIANCE POSITIONS (Y > 0, toward Blue Wall) ===
    
    // Start positions
    public static final FieldPose BLUE_START_LEFT = new FieldPose(-48.0, 60.0, 270.0);
    public static final FieldPose BLUE_START_RIGHT = new FieldPose(48.0, 60.0, 270.0);
    
    // Artifact intake positions (3 rows of coral)
    public static final FieldPose BLUE_INTAKE_1_START = new FieldPose(-36.0, 48.0, 270.0);
    public static final FieldPose BLUE_INTAKE_1_FINISH = new FieldPose(-36.0, 36.0, 270.0);
    public static final FieldPose BLUE_INTAKE_2_START = new FieldPose(0.0, 36.0, 270.0);
    public static final FieldPose BLUE_INTAKE_2_FINISH = new FieldPose(0.0, 24.0, 270.0);
    public static final FieldPose BLUE_INTAKE_3_START = new FieldPose(36.0, 24.0, 270.0);
    public static final FieldPose BLUE_INTAKE_3_FINISH = new FieldPose(36.0, 12.0, 270.0);
    
    // Shooting positions (targeting Red GOAL)
    public static final FieldPose BLUE_SHOOTING_CLOSE = new FieldPose(0.0, 48.0, 180.0);
    public static final FieldPose BLUE_SHOOTING_MID = new FieldPose(0.0, 36.0, 180.0);
    public static final FieldPose BLUE_SHOOTING_LONG = new FieldPose(0.0, 24.0, 180.0);
    
    // Utility positions
    public static final FieldPose BLUE_BASE_ZONE = new FieldPose(-60.0, 60.0, 315.0);
    public static final FieldPose BLUE_LOADING_ZONE = new FieldPose(-60.0, 36.0, 180.0);
    public static final FieldPose BLUE_SECRET_TUNNEL = new FieldPose(0.0, 72.0, 270.0);
    public static final FieldPose BLUE_GATE_ZONE = new FieldPose(12.0, 72.0, 270.0);
    public static final FieldPose BLUE_PARKING = new FieldPose(-48.0, 48.0, 315.0);
    
    // === APRILTAG & OBELISK POSITIONS ===
    
    // GOAL positions (for distance calculations and targeting)
    public static final FieldPose RED_GOAL = new FieldPose(0.0, -72.0, 90.0);             // Red Alliance GOAL
    public static final FieldPose BLUE_GOAL = new FieldPose(0.0, 72.0, 270.0);            // Blue Alliance GOAL
    
    // GOAL AprilTags (for shooting targeting)
    public static final FieldPose RED_GOAL_APRILTAG = new FieldPose(0.0, -72.0, 90.0);    // Tag 24
    public static final FieldPose BLUE_GOAL_APRILTAG = new FieldPose(0.0, 72.0, 270.0);   // Tag 20
    
    // OBELISK AprilTags (position varies - approximate center outside Blue wall)
    public static final FieldPose OBELISK_POSITION = new FieldPose(0.0, 72.0, 270.0);
    public static final FieldPose OBELISK_TAG_21 = new FieldPose(0.0, 72.0, 270.0);       // MOTIF face 1
    public static final FieldPose OBELISK_TAG_22 = new FieldPose(0.0, 72.0, 270.0);       // MOTIF face 2
    public static final FieldPose OBELISK_TAG_23 = new FieldPose(0.0, 72.0, 270.0);       // MOTIF face 3
    
    // ========== INITIALIZATION ==========
    
    static {
        // Register DECODE field positions in the map for consistent access
        
        // Red Alliance positions
        put("red_start_left", RED_START_LEFT);
        put("red_start_right", RED_START_RIGHT);
        put("red_intake_1_start", RED_INTAKE_1_START);
        put("red_intake_1_finish", RED_INTAKE_1_FINISH);
        put("red_intake_2_start", RED_INTAKE_2_START);
        put("red_intake_2_finish", RED_INTAKE_2_FINISH);
        put("red_intake_3_start", RED_INTAKE_3_START);
        put("red_intake_3_finish", RED_INTAKE_3_FINISH);
        put("red_shooting_close", RED_SHOOTING_CLOSE);
        put("red_shooting_mid", RED_SHOOTING_MID);
        put("red_shooting_long", RED_SHOOTING_LONG);

        put("red_base_zone", RED_BASE_ZONE);
        put("red_loading_zone", RED_LOADING_ZONE);
        put("red_secret_tunnel", RED_SECRET_TUNNEL);
        put("red_gate_zone", RED_GATE_ZONE);
        put("red_parking", RED_PARKING);
        
        // Blue Alliance positions
        put("blue_start_left", BLUE_START_LEFT);
        put("blue_start_right", BLUE_START_RIGHT);
        put("blue_intake_1_start", BLUE_INTAKE_1_START);
        put("blue_intake_1_finish", BLUE_INTAKE_1_FINISH);
        put("blue_intake_2_start", BLUE_INTAKE_2_START);
        put("blue_intake_2_finish", BLUE_INTAKE_2_FINISH);
        put("blue_intake_3_start", BLUE_INTAKE_3_START);
        put("blue_intake_3_finish", BLUE_INTAKE_3_FINISH);
        put("blue_shooting_close", BLUE_SHOOTING_CLOSE);
        put("blue_shooting_mid", BLUE_SHOOTING_MID);
        put("blue_shooting_long", BLUE_SHOOTING_LONG);

        put("blue_base_zone", BLUE_BASE_ZONE);
        put("blue_loading_zone", BLUE_LOADING_ZONE);
        put("blue_secret_tunnel", BLUE_SECRET_TUNNEL);
        put("blue_gate_zone", BLUE_GATE_ZONE);
        put("blue_parking", BLUE_PARKING);
        
        // GOAL positions
        put("red_goal", RED_GOAL);
        put("blue_goal", BLUE_GOAL);
        
        // AprilTag and OBELISK positions
        put("red_goal_apriltag", RED_GOAL_APRILTAG);
        put("blue_goal_apriltag", BLUE_GOAL_APRILTAG);
        put("obelisk_position", OBELISK_POSITION);
        put("obelisk_tag_21", OBELISK_TAG_21);
        put("obelisk_tag_22", OBELISK_TAG_22);
        put("obelisk_tag_23", OBELISK_TAG_23);
    }
    
    // ========== POSITION MANAGEMENT ==========
    
    /**
     * Gets a named position
     * 
     * @param name Position name (case-insensitive)
     * @return FieldPose for the named position, or null if not found
     */
    public static FieldPose get(String name) {
        return positions.get(name.toLowerCase());
    }
    
    /**
     * Adds or updates a named position
     * 
     * @param name Position name (will be stored in lowercase)
     * @param pose FieldPose to associate with the name
     */
    public static void put(String name, FieldPose pose) {
        positions.put(name.toLowerCase(), new FieldPose(pose));
    }
    
    /**
     * Adds or updates a named position with coordinates
     * 
     * @param name Position name (will be stored in lowercase)
     * @param x Field X coordinate (inches)
     * @param y Field Y coordinate (inches)
     * @param heading Robot heading (degrees)
     */
    public static void put(String name, double x, double y, double heading) {
        positions.put(name.toLowerCase(), new FieldPose(x, y, heading));
    }
    
    /**
     * Checks if a named position exists
     * 
     * @param name Position name to check
     * @return true if position exists, false otherwise
     */
    public static boolean contains(String name) {
        return positions.containsKey(name.toLowerCase());
    }
    
    /**
     * Removes a named position
     * 
     * @param name Position name to remove
     * @return true if position was removed, false if it didn't exist
     */
    public static boolean remove(String name) {
        return positions.remove(name.toLowerCase()) != null;
    }
    
    /**
     * Gets all position names
     * 
     * @return Array of all position names
     */
    public static String[] getAllNames() {
        return positions.keySet().toArray(new String[0]);
    }
    
    /**
     * Clears all positions (including predefined ones)
     * Use with caution - typically only needed for testing
     */
    public static void clear() {
        positions.clear();
    }
    
    // ========== FILE LOADING ==========
    
    /**
     * Loads positions from a properties-style file.
     * 
     * File format:
     * ```
     * # Comments start with #
     * position_name=x,y,heading
     * high_basket=48.0,60.0,45.0
     * start_pos=12.0,12.0,0.0
     * ```
     * 
     * @param filename Path to the properties file
     * @return Number of positions loaded, or -1 if file couldn't be read
     */
    public static int loadFromFile(String filename) {
        int loadedCount = 0;
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            int lineNumber = 0;
            
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                line = line.trim();
                
                // Skip empty lines and comments
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                // Parse line: name=x,y,heading
                String[] parts = line.split("=", 2);
                if (parts.length != 2) {
                    System.err.println("FieldPositions: Invalid format at line " + lineNumber + ": " + line);
                    continue;
                }
                
                String name = parts[0].trim();
                String[] coords = parts[1].trim().split(",");
                
                if (coords.length != 3) {
                    System.err.println("FieldPositions: Invalid coordinates at line " + lineNumber + ": " + parts[1]);
                    continue;
                }
                
                try {
                    double x = Double.parseDouble(coords[0].trim());
                    double y = Double.parseDouble(coords[1].trim());
                    double heading = Double.parseDouble(coords[2].trim());
                    
                    put(name, x, y, heading);
                    loadedCount++;
                    
                } catch (NumberFormatException e) {
                    System.err.println("FieldPositions: Invalid number format at line " + lineNumber + ": " + parts[1]);
                }
            }
            
        } catch (IOException e) {
            System.err.println("FieldPositions: Could not read file " + filename + ": " + e.getMessage());
            return -1;
        }
        
        return loadedCount;
    }
    
    // ========== UTILITY METHODS ==========
    
    /**
     * Creates an array of FieldPoses from named positions.
     * Useful for creating waypoint paths.
     * 
     * @param names Position names in order
     * @return Array of FieldPoses, or null if any name is not found
     */
    public static FieldPose[] createPath(String... names) {
        FieldPose[] path = new FieldPose[names.length];
        
        for (int i = 0; i < names.length; i++) {
            FieldPose pose = get(names[i]);
            if (pose == null) {
                System.err.println("FieldPositions: Position not found: " + names[i]);
                return null;
            }
            path[i] = new FieldPose(pose);  // Create copy to avoid modification
        }
        
        return path;
    }
    
    /**
     * Finds the closest position to the given coordinates
     * 
     * @param x Target X coordinate
     * @param y Target Y coordinate
     * @return Name of closest position, or null if no positions exist
     */
    public static String findClosest(double x, double y) {
        if (positions.isEmpty()) {
            return null;
        }
        
        String closestName = null;
        double closestDistance = Double.MAX_VALUE;
        FieldPose target = new FieldPose(x, y);
        
        for (Map.Entry<String, FieldPose> entry : positions.entrySet()) {
            double distance = entry.getValue().distanceTo(target);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestName = entry.getKey();
            }
        }
        
        return closestName;
    }
    
    /**
     * Gets a summary of all loaded positions
     * 
     * @return String summary of all positions
     */
    public static String getSummary() {
        if (positions.isEmpty()) {
            return "No positions loaded";
        }
        
        StringBuilder summary = new StringBuilder();
        summary.append("Loaded Positions (").append(positions.size()).append("):\n");
        
        for (Map.Entry<String, FieldPose> entry : positions.entrySet()) {
            summary.append("  ").append(entry.getKey()).append(": ").append(entry.getValue()).append("\n");
        }
        
        return summary.toString();
    }
}

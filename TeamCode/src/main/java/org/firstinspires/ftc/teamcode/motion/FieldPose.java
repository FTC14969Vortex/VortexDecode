package org.firstinspires.ftc.teamcode.motion;

/**
 * Represents a position and orientation on the field in reference point coordinates.
 * 
 * This class provides a simple data structure for field positions where:
 * - x, y coordinates represent where the robot's reference point should be positioned
 * - heading represents the robot's orientation at that position
 * 
 * The reference point is configured in MotionConfig.ACTIVE_REFERENCE_POINT and could be:
 * - SCORING_POINT: For positioning the scoring mechanism
 * - INTAKE_POINT: For positioning the intake mechanism  
 * - Robot center: For traditional robot-centric positioning
 * 
 * All motion commands that accept FieldPose will automatically convert these reference
 * point coordinates to robot center coordinates internally.
 */
public class FieldPose {
    /** Field X coordinate where reference point should be positioned (inches) */
    public double x;
    
    /** Field Y coordinate where reference point should be positioned (inches) */
    public double y;
    
    /** Robot heading at this position (degrees, 0 = facing +X field axis) */
    public double heading;
    
    /**
     * Creates a new field pose with specified position and heading
     * 
     * @param x Field X coordinate (inches)
     * @param y Field Y coordinate (inches) 
     * @param heading Robot heading (degrees)
     */
    public FieldPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    
    /**
     * Creates a new field pose with specified position and default heading (0 degrees)
     * 
     * @param x Field X coordinate (inches)
     * @param y Field Y coordinate (inches)
     */
    public FieldPose(double x, double y) {
        this(x, y, 0.0);
    }
    
    /**
     * Copy constructor
     * 
     * @param other FieldPose to copy
     */
    public FieldPose(FieldPose other) {
        this(other.x, other.y, other.heading);
    }
    
    /**
     * Calculates the distance between this pose and another pose (ignoring heading)
     * 
     * @param other Other pose to calculate distance to
     * @return Distance in inches
     */
    public double distanceTo(FieldPose other) {
        double deltaX = other.x - this.x;
        double deltaY = other.y - this.y;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    /**
     * Calculates the angle from this pose to another pose
     * 
     * @param other Other pose to calculate angle to
     * @return Angle in degrees (0 = +Y axis, 90 = +X axis)
     */
    public double angleTo(FieldPose other) {
        double deltaX = other.x - this.x;
        double deltaY = other.y - this.y;
        return Math.toDegrees(Math.atan2(deltaX, deltaY));
    }
    
    /**
     * Creates a new pose offset from this pose by the specified amounts
     * 
     * @param deltaX X offset (inches)
     * @param deltaY Y offset (inches)
     * @param deltaHeading Heading offset (degrees)
     * @return New FieldPose offset from this one
     */
    public FieldPose offset(double deltaX, double deltaY, double deltaHeading) {
        return new FieldPose(this.x + deltaX, this.y + deltaY, this.heading + deltaHeading);
    }

    public FieldPose mirror(String axis_name) {
        if (axis_name.equals("x") || axis_name.equals("X")) {
            return new FieldPose(this.x, -this.y, -this.heading); // also change heading
        } else if (axis_name.equals("y") || axis_name.equals("Y")) {
            return new FieldPose(-this.x, this.y, 180-this.heading);
        } else {
            return new FieldPose(this.x, this.y, this.heading);
        }
    }


    
    /**
     * Creates a new pose offset from this pose by the specified distance and angle
     * 
     * @param distance Distance to offset (inches)
     * @param angle Angle of offset (degrees, 0 = +Y axis)
     * @return New FieldPose offset from this one
     */
    public FieldPose offsetPolar(double distance, double angle) {
        double angleRad = Math.toRadians(angle);
        double deltaX = distance * Math.sin(angleRad);  // sin for X because 0 degrees = +Y axis
        double deltaY = distance * Math.cos(angleRad);  // cos for Y because 0 degrees = +Y axis
        return new FieldPose(this.x + deltaX, this.y + deltaY, this.heading);
    }
    
    @Override
    public String toString() {
        return String.format("FieldPose(%.1f, %.1f, %.1f deg)", x, y, heading);
    }
    
    /**
     * Returns a compact string representation
     * 
     * @return Compact string representation
     */
    public String toCompactString() {
        return String.format("(%.1f,%.1f,%.1f deg)", x, y, heading);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        
        FieldPose fieldPose = (FieldPose) obj;
        return Double.compare(fieldPose.x, x) == 0 &&
               Double.compare(fieldPose.y, y) == 0 &&
               Double.compare(fieldPose.heading, heading) == 0;
    }
    
    @Override
    public int hashCode() {
        long temp = Double.doubleToLongBits(x);
        int result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(heading);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}

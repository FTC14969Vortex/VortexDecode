package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Flipper servo control class that supports both 4.5-turn and 1-turn (300 degree) servos.
 *
 * Servo Types:
 * - 4.5 Turn Servo: Full range = 0.0 to 1.0 maps to 0 degrees to 1620 degrees (4.5 x 360 degrees)
 * - 1 Turn Servo (300 degrees): Full range = 0.0 to 1.0 maps to 0 degrees to 300 degrees
 *
 * Configuration: Set SERVO_TYPE to match your hardware
 */
public class Flipper {
    private Servo flipper;
    private OpMode opMode;

    // ===== SERVO CONFIGURATION =====
    // Change this enum value to match your servo type
    public enum ServoType {
        TURN_4_5,    // 4.5 turn servo (1620 degrees total range)
        TURN_1_300   // 1 turn servo with 300 degree range
    }

    // ***** SET YOUR SERVO TYPE HERE *****
    //private static final ServoType SERVO_TYPE = ServoType.TURN_4_5; // 4.5-turn servo
     private static final ServoType SERVO_TYPE = ServoType.TURN_1_300; // 1-turn (300 degrees) servo

    // Servo specifications
    private static final double SERVO_4_5_TURN_DEGREES = 1620.0;  // 4.5 x 360 degrees
    private static final double SERVO_1_TURN_DEGREES = 300.0;      // 300 degrees range

    // Starting position (0.0 to 1.0 servo position)
    public static final double FLIPPER_START_POSITION = 0.5;  // Start at center position

    // Calculated degrees per servo position unit (0.0-1.0 range)
    private final double degreesPerUnit;

    public Flipper() {
        // Calculate degrees per unit based on servo type
        if (SERVO_TYPE == ServoType.TURN_4_5) {
            degreesPerUnit = SERVO_4_5_TURN_DEGREES;  // 1620 degrees per unit
        } else {
            degreesPerUnit = SERVO_1_TURN_DEGREES;    // 300 degrees per unit
        }
    }

    public void init(HardwareMap hwMap) {
        flipper = hwMap.get(Servo.class, "flipper");
        // opMode field is not used, no need to set it
        resetFlipper();
    }


    public void setPosition(double position){
        // Clamp position to valid range
        position = Math.max(0.0, Math.min(1.0, position));
        flipper.setPosition(position);
    }

    /**
     * Resets the flipper to its starting position.
     */
    public void resetFlipper() {
        setPosition(FLIPPER_START_POSITION);
    }

    /**
     * Turns the flipper to a specific angle in degrees.
     * The angle is measured from the zero position (servo position 0.0).
     *
     * For 4.5 turn servo: Valid range is 0 degrees to 1620 degrees, uses inverted formula (1 - angle/totalDegrees)
     * For 1 turn (300 degrees) servo: Valid range is 0 degrees to 300 degrees, uses direct formula (angle/totalDegrees)
     *
     */
    public void turnFlipper(double angle) {
        double targetPosition;

        targetPosition = FLIPPER_START_POSITION - (angle / degreesPerUnit);

        // Clamp to valid range and set position
        setPosition(targetPosition);
    }

    /**
     * Returns the last commanded servo position (0.0 to 1.0).
     * Note: This does NOT return the actual physical position, only the last commanded value.
     */
    public double getPosition(){
        return flipper.getPosition();
    }

    /**
     * Returns the last commanded angle in degrees based on the servo type.
     * Note: This does NOT return the actual physical angle, only the calculated value
     * from the last commanded position.

     */
    public double getAngleDegrees() {
        return getPosition() * degreesPerUnit;
    }

    /**
     * Returns the servo type being used.
     *
     */
    public ServoType getServoType() {
        return SERVO_TYPE;
    }

    /**
     * Returns the maximum angle range for the configured servo type.
     *

     */
    public double getMaxAngleDegrees() {
        return degreesPerUnit;
    }

}

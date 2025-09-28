package org.firstinspires.ftc.teamcode.helper;

public class Util {
    public static double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }
    /**
     * Clips a value to be within a minimum and maximum range.
     */
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static void sleepThread(int millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }
}

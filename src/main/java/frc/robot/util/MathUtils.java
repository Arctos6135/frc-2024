package frc.robot.util;

/**
 * This class contains methods to preform common math tasks.
 */
public class MathUtils {
    /**
     * Makes number at most max, and at least min.
     * @param number
     * @param min
     * @param max
     * @return clamped number
     */
    public static double clamp(double number, double min, double max) {
        return Math.max(min, Math.min(number, max));
    }


    /**
     * Return True if the difference between num1 and num2 is negligible.
     * @param num1
     * @param num2
     * @return
     */
    public static Boolean closeEnough(double num1, double num2) {
        // Value to which the the difference is negligible.
        double distance = 0.01;

        return Math.abs(num1 - num2) <= distance;
    }
}

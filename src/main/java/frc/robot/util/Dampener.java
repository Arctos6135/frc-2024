package frc.robot.util;

public class Dampener {
    // a value between 0 and 1, where 0 means no dampening and 1 means a x^3 curve
    private final double nonlinearity;

    public Dampener(double nonlinearity) {
        this.nonlinearity = nonlinearity;
    }

    public double dampen(double input) {
        return nonlinearity * Math.pow(input, 3) + (1 - nonlinearity) * input;
    }
}

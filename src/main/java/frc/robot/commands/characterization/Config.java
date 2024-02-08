package frc.robot.commands.characterization;

public class Config {
    public final double maxDistance;
    public final double voltageRampRate;
    public final double maxVoltage;

    public Config(double maxDistance, double voltageRampRate, double maxVoltage) {
        this.maxDistance = maxDistance;
        this.voltageRampRate = voltageRampRate;
        this.maxVoltage = maxVoltage;
    }

    public Config(double maxDistance) {
        this(maxDistance, 1, 6);
    }
}

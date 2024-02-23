package frc.robot.commands.characterization;

public class SensorData {
    public final double distance;
    public final double velocity;
    public final double acceleration;

    public SensorData(double distance, double velocity, double acceleration) {
        this.distance = distance;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
}

package frc.robot.commands.characterization;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

public class Mechanism {
    private final DoubleConsumer voltageSetter;
    private final DoubleSupplier positionGetter;
    private final DoubleSupplier velocityGetter;
    private double startPosition;
    private double previousVelocity;
    private double previousVelocityTimestamp;

    public Mechanism(DoubleConsumer voltageSetter, DoubleSupplier positionGetter, DoubleSupplier velocityGetter) {
        this.voltageSetter = voltageSetter;
        this.positionGetter = positionGetter;
        this.velocityGetter = velocityGetter;
    }

    public void initialize() {
        startPosition = positionGetter.getAsDouble();
        previousVelocity = velocityGetter.getAsDouble();
        previousVelocityTimestamp = Timer.getFPGATimestamp();
    }

    public SensorData readSensors() {
        double distance = positionGetter.getAsDouble() - startPosition;
        double velocity = velocityGetter.getAsDouble();
        double timestamp = Timer.getFPGATimestamp();
        double acceleration = (velocity - previousVelocity) / (timestamp - previousVelocityTimestamp);
        previousVelocity = velocity;
        previousVelocityTimestamp = timestamp;
        
        return new SensorData(distance, velocity, acceleration);
    }

    public void setVoltage(double voltage) {
        voltageSetter.accept(voltage);
    }
}

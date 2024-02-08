package frc.robot.commands.characterization;

public class LoggedMechanism {
    private final Logger logger;
    private final Mechanism mechanism;
    private double previousVoltage = 0;

    public LoggedMechanism(Logger logger, Mechanism mechanism) {
        this.logger = logger;
        this.mechanism = mechanism;
    }

    public double update(double voltage) {
        SensorData data = mechanism.readSensors();
        logger.accept(previousVoltage, data.velocity, data.acceleration);
        previousVoltage = voltage;
        mechanism.setVoltage(voltage);
        return data.distance;
    }
}

package frc.robot.commands.characterization;

public class LoggedMechanism {
    private final FeedforwardLog logger;
    private final Mechanism mechanism;
    private double previousVoltage = 0;

    public LoggedMechanism(FeedforwardLog logger, Mechanism mechanism) {
        this.logger = logger;
        this.mechanism = mechanism;
    }

    public void initialize() {
        mechanism.initialize();
    }

    public double update(double voltage) {
        SensorData data = mechanism.readSensors();
        logger.accept(previousVoltage, data.velocity, data.acceleration);
        previousVoltage = voltage;
        mechanism.setVoltage(voltage);
        return data.distance;
    }
}

package frc.robot.commands.characterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class VelocityRoutine extends Command {
    private final LoggedMechanismGroup mechanism;
    private final double maxDistance;
    private final double rampRate;
    private double voltage = 0;
    private double lastTimestamp;

    public VelocityRoutine(LoggedMechanismGroup mechanism, double maxDistance, double rampRate, Subsystem... requirements) {
        this.mechanism = mechanism;
        this.maxDistance = maxDistance;
        this.rampRate = rampRate;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double timestamp = Timer.getFPGATimestamp();
        voltage += rampRate * (timestamp - lastTimestamp);
        lastTimestamp = timestamp;

        mechanism.setVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        boolean tooFar = mechanism.getGreatestDistance() > maxDistance;
        boolean tooFarBack = mechanism.getLeastDistance() < 0;

        return tooFar || tooFarBack;
    }

    @Override
    public void end(boolean interupted) {
        mechanism.setVoltage(0);
    }
}
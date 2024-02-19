package frc.robot.commands.characterization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AccelerationRoutine extends Command {
    private final LoggedMechanismGroup mechanism;
    private final double maxDistance;
    private final double voltage;

    public AccelerationRoutine(LoggedMechanismGroup mechanism, double maxDistance, double voltage, Subsystem... requirements) {
        this.mechanism = mechanism;
        this.maxDistance = maxDistance;
        this.voltage = voltage;

        addRequirements(requirements);
    }

    @Override
    public void execute() {
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
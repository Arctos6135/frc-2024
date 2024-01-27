package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Allow robot to start shooting. Does not have a built-in isFinished condition.
 */
public class Launch extends Command {
    Shooter shooter;

    public Launch(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVoltages(ShooterConstants.LEFT_VOLTAGE, ShooterConstants.RIGHT_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean disrupted) {
        shooter.setVoltages(0, 0);
    }
}

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Allow robot to start shooting. Does not have a built-in isFinished condition.
 */
public class Launch extends Command {
    private final Shooter shooter;

    // Rotations per second
    private double speed;

    /**
     * Call the launch command.
     * @param shooter the shooter subsystem
     * @param speed the speed at which the wheels should rotate at. In rotations per second.
     */
    public Launch(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setRPS(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean disrupted) {
        shooter.stop();
    }
}

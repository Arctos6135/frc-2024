package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CurrentFeed extends Command{
    private final Intake intake;
    private final Shooter shooter;

    public CurrentFeed(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
        shooter.setRPS(ShooterConstants.FEED_RPS);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return intake.getFilteredCurrent() <= IntakeConstants.INTAKE_CURRENT;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
        shooter.setRPS(0);
    }
}

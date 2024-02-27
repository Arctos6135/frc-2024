package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CurrentFeed extends Command{
    private final Intake intake;
    private final Shooter shooter;
    private double maxCurrent;

    private double startTime;

    public CurrentFeed(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        maxCurrent = 0;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
        shooter.setRPS(ShooterConstants.FEED_RPS);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double current = intake.getFilteredCurrent();
        if (current > maxCurrent) {
            maxCurrent = current;
        }
    }

    @Override
    public boolean isFinished() {
        return maxCurrent - intake.getFilteredCurrent() >= 3 && (Timer.getFPGATimestamp() - startTime) > 0.25;
        // return intake.getFilteredCurrent() <= 8 && (Timer.getFPGATimestamp() - startTime) > 0.25;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
        shooter.setRPS(0);
    }
}

package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class RaceFeed extends Command{
    private final Intake intake;

    public RaceFeed(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

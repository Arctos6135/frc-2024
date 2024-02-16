package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CurrentFeed extends Command{
    private final Intake intake;

    public CurrentFeed(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
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
    }
}

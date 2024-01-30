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

    }

    @Override
    public void execute() {
        intake.setVoltage(IntakeConstants.VOLTAGE);
    }

    @Override
    public boolean isFinished() {
    
        return (
            intake.getFilteredCurrent() >= (IntakeConstants.VOLTAGE / IntakeConstants.STANDARD_RESISTANCE) - IntakeConstants.STANDARD_CURRENT_ERROR 
            &&
            intake.getFilteredCurrent() <= (IntakeConstants.VOLTAGE / IntakeConstants.STANDARD_RESISTANCE) + IntakeConstants.STANDARD_CURRENT_ERROR
        );
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

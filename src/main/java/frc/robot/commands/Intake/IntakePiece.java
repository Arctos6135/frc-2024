package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;


/**
 * Allow robot to intake notes. 
 * 
 * This command uses the current spikes of the motor to 
 * determine when a note has been successfull intaken.
 */
public class IntakePiece extends Command {
    private final Intake intake;


    public IntakePiece(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.setVoltage(IntakeConstants.VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return intake.getFilteredCurrent() >= IntakeConstants.MAX_EMPTY_CURRENT;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

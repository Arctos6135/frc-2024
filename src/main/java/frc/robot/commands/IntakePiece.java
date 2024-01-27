package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakePiece extends Command {
    private final Intake intake;


    public IntakePiece(Intake intake) {
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
        if (intake.getFilteredCurrent() >= IntakeConstants.MAX_CURRENT){
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

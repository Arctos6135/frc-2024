package frc.robot.commands.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.TunableNumber;


/**
 * Allow robot to intake notes. 
 * 
 * This command uses the current spikes of the motor to 
 * determine when a note has been successfull intaken.
 */
public class IntakePieceSpeed extends Command {
    private final Intake intake;
    private final MedianFilter speedFilter;
    private final TunableNumber speedDrop = new TunableNumber("Intake/Speed Drop", 0.5);
    private double normalSpeed = 0;

    public IntakePieceSpeed(Intake intake) {
        this.intake = intake;
        speedFilter = new MedianFilter(5);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        speedFilter.reset();
        normalSpeed = 0;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Input/Median Velocity", normalSpeed);
        normalSpeed = speedFilter.calculate(intake.getVelocity());
        intake.setVoltage(IntakeConstants.VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return intake.getVelocity() < (normalSpeed - speedDrop.get());
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

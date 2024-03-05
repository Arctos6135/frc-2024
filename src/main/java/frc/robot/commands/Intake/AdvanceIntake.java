package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class AdvanceIntake extends Command {
    private Intake intake;
    private double distance;
    private boolean forwards;
    private double startPosition;
    
    /**
     * 
     * 
     * @param intake the intake 
     * @param distance distance to move the intake in meters can take negative
     */ 
    public void AdvanceIntake(Intake intake, double distance) {
        this.intake = intake;
        this.distance = distance;
        forwards = distance > 0;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        startPosition = intake.getPosition();
        if (!forwards) {intake.setVoltage(-IntakeConstants.FEED_VOLTAGE);}
        else {intake.setVoltage(IntakeConstants.FEED_VOLTAGE);}
    }

    @Override
    public boolean isFinished() {
        double distanceTraveled = Math.abs(intake.getPosition() - startPosition);
        return Math.abs(distance) < distanceTraveled;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

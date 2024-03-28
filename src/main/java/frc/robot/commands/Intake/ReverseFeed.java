package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ReverseFeed extends Command{
    private final Shooter shooter;
    private final Intake intake;

    private double meters;

    private double initialPosition;

    /**
     * 
     * @param shooter the shooter subsystem
     * @param meters the 
     */
    public ReverseFeed(Shooter shooter, Intake intake, double meters){
        this.shooter = shooter;
        this.intake = intake;
        this.meters = meters;
        this.initialPosition = shooter.getPosition();

        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.setVoltage(-ShooterConstants.ADVANCE_VOLTAGE);
        intake.setVoltage(-IntakeConstants.FEED_VOLTAGE);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Math.abs(shooter.getPosition()) - Math.abs(initialPosition)) >= meters;
    }

    @Override
    public void end(boolean disrupted){
        shooter.stop();
        intake.setVoltage(0);
    }
}


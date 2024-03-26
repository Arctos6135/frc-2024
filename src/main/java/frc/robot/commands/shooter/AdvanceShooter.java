package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.constants.ShooterConstants;

public class AdvanceShooter extends Command{
    private final Shooter shooter;

    private double meters;

    private double initialPosition;

    /**
     * 
     * @param shooter the shooter subsystem
     * @param meters the 
     */
    public AdvanceShooter(Shooter shooter, double meters){
        this.shooter = shooter;
        this.meters = meters;
        this.initialPosition = shooter.getPosition();

        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.setVoltage(ShooterConstants.ADVANCE_VOLTAGE);
    }

    @Override
    public boolean isFinished(){
        return shooter.getPosition() - this.initialPosition >= meters;
    }

    @Override
    public void end(boolean disrupted){
        shooter.stop();
    }
}

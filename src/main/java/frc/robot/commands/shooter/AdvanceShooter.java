package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.constants.ShooterConstants;

public class AdvanceShooter extends Command{
    private final Shooter shooter;

    private double distance;

    private double initialPosition;

    /**
     * 
     * @param shooter the shooter subsystem
     * @param distance the number of radians we want to rotate shooter wheels by
     */
    public AdvanceShooter(Shooter shooter, Double distance){
        this.shooter = shooter;
        this.distance = distance;
        this.initialPosition = shooter.getPosition();

        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.setVoltages(ShooterConstants.ADVANCE_VOLTAGE, ShooterConstants.ADVANCE_VOLTAGE);
    }

    @Override
    public boolean isFinished(){
        return shooter.getPosition() - this.initialPosition >= distance / (2.0 * Math.PI);
    }

    @Override
    public void end(boolean disrupted){
        shooter.stop();
    }
}

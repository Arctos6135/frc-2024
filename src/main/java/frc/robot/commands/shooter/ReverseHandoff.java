package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ReverseHandoff extends Command{
    private final Shooter shooter;

    private double meters;

    private double initialPosition;

    /**
     * 
     * @param shooter the shooter subsystem
     * @param meters the 
     */
    public ReverseHandoff(Shooter shooter, double meters){
        this.shooter = shooter;
        this.meters = meters;
        this.initialPosition = shooter.getPosition();

        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.setVoltages(-ShooterConstants.ADVANCE_VOLTAGE);
    }

    @Override
    public boolean isFinished(){
        return this.initialPosition - shooter.getPosition() >= meters;
    }

    @Override
    public void end(boolean disrupted){
        shooter.stop();
    }
}


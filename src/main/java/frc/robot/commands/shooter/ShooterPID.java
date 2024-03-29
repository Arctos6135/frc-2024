package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TunableNumber;

public class ShooterPID extends Command{
    private final Shooter shooter;

    private final TunableNumber kP = new TunableNumber("Shooter/kP", 0.00002);
    private final TunableNumber kI = new TunableNumber("Shooter/kI", 0);
    private final TunableNumber kD = new TunableNumber("Shooter/kD", 0);

    private double targetVelocity;
    
    public ShooterPID(Shooter shooter, double targetVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;

        TunableNumber.ifChanged(
            () -> {
                shooter.calibratePIDController(kP.get(), kI.get(), kD.get());
            },
            kP,
            kI,
            kD
        );
    }

    @Override
    public void execute(){
        Logger.recordOutput("Shooter/Setpoint Velocity", targetVelocity);
        shooter.setPIDTargetVelocity(targetVelocity);
    }

    public boolean atTarget() {
        return Math.abs(shooter.getVelocity() - targetVelocity) < 0.5;
    }

    @Override
    public void end(boolean i) {
        shooter.setPIDTargetVelocity(0);
    }
        
}

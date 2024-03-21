package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TunableNumber;

public class ShooterPID extends Command{
    private final Shooter shooter;

    private final TunableNumber kFF = new TunableNumber("Shooter/kFF", 0.5);
    private final TunableNumber kP = new TunableNumber("Shooter/kP", 7);
    private final TunableNumber kI = new TunableNumber("Shooter/kI", 14);
    private final TunableNumber kD = new TunableNumber("Shooter/kD", 7);

    private double targetSpeed;
    
    public ShooterPID(Shooter shooter, double targetSpeed) {
        this.shooter = shooter;
        this.targetSpeed = targetSpeed;

        TunableNumber.ifChanged(
            () -> {
                shooter.calibratePIDController(kP.get(), kI.get(), kD.get(), kFF.get());
            },
            kP,
            kI,
            kD,
            kFF
        );
    }

    @Override
    public void execute(){
        shooter.setPIDTargetVelocity(targetSpeed);
    }
        
}

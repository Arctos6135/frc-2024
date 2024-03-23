package frc.robot.commands.shooter;

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

    private double leftTargetSpeed;
    private double rightTargetSpeed;
    
    public ShooterPID(Shooter shooter, double leftTargetSpeed, double rightTargetSpeed) {
        this.shooter = shooter;
        this.leftTargetSpeed = leftTargetSpeed;
        this.rightTargetSpeed = rightTargetSpeed;

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
        shooter.setPIDTargetVelocities(leftTargetSpeed, rightTargetSpeed);
    }

    // public boolean atTarget() {
    //     // NOTE: 0.1 is totally arbitrary, we need to come up with a reasonale value
    //     // although we currently arent using atTarget() so not urgent
    //     return Math.abs(shooter.getVelocity() - targetSpeed) < 0.1;
    // }

    @Override
    public void end(boolean i) {
        shooter.setPIDTargetVelocities(0, 0);
    }
        
}

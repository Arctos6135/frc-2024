package frc.robot.commands.driving;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.TunableNumber;
import frc.robot.util.MathUtils;

/**
 * NOTES:
 * Drivetrain should have a `.getYaw()` method that provides information
 * about the current yaw
 * 
 * We should have a RotationPIDController in the Drivetrain that
 * every command uses.
 * This should be accessible through the `.getRotationController()`
 * method (or something similar)
 * 
 * Drivetrain should have a method with the following signature:
 *  ```arcadeDrive(speed: double, rotation: double);```
 */

/**
 * A command that exposes the functionality to orient the drivetrain to
 * a specific yaw relative to the original starting position of the robot
 * at execution.
 */
public class PIDSetAngle extends Command {
    private final Drivetrain drivetrain;

    private final TunableNumber kP = new TunableNumber("PIDSetAngle kP", 7);
    private final TunableNumber kI = new TunableNumber("PIDSetAngle kI", 0);
    private final TunableNumber kD = new TunableNumber("PIDSetAngle kD", 0.1);

    private final PIDController rotationController = new PIDController(0, 0, 0);

    private final double setpointAngle;

    /**
     * @param speed The speed at which the motor  
     * @param angle The angle the robot should turn towards.
     * Range is [0, 2Pi]
     */
    public PIDSetAngle(Drivetrain drivetrain, double setpointAngle) {
        this.drivetrain = drivetrain;
        this.setpointAngle = setpointAngle;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        rotationController.setPID(kP.get(), kI.get(), kD.get());
        // Get the rotation speed at which we should be rotating
        double pidRotation = rotationController.calculate(
            drivetrain.getYaw(), // Current Rotation
            setpointAngle
        );

        Logger.recordOutput("Drivetrain/Proportional SetAngle", kP.get() * (drivetrain.getYaw() - setpointAngle));
        rotationController.setIntegratorRange(-12, 12);

        // Clamp the speed
        pidRotation = MathUtils.clamp(pidRotation, -10, 10);
        Logger.recordOutput("Drivetrain/Total Set Angle", pidRotation);

        drivetrain.arcadeDrive(0, pidRotation);

        Logger.recordOutput("Drivetrain/Setting Angle", true);
        Logger.recordOutput("Drivetrain/Target Yaw", setpointAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getYaw() - setpointAngle) < DriveConstants.ROTATION_TOLERANCE && drivetrain.getYawRate() < DriveConstants.ROTATION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Drivetrain/Setting Angle", false);
        drivetrain.arcadeDrive(0, 0);
    }
}

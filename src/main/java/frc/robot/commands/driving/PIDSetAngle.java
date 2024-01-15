import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

package frc.robot.commands.driving;

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

    private final PIDController rotationController;

    private double startingAngle;
    private double setpointAngle;

    /**
     * @param speed The speed at which the motor  
     * @param angle The angle the robot should turn towards.
     * Range is [0, 2Pi]
     */
    public PIDSetAngle(Drivetrain drivetrain, double setpointAngle) {
        this.drivetrain = drivetrain;
        this.rotationController = drivetrain.getRotationController();

        this.setpointAngle = setpointAngle;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void execute() {
        // Get the rotation speed at which we should be rotating
        double pidRotation = rotationController.calculate(
            drivetrain.getYaw(), // Current Rotation
            setpointAngle
        );

        // Clamp the speed
        pidRotation = Math.min(0.5, Math.max(pidRotation, -0.5));

        drivetrain.arcadeDrive(0, pidRotation);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getYaw() - setpointAngle) < DrivetrainConstants.ROTATION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}

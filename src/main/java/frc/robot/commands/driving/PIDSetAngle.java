package frc.robot.commands.driving;

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

    private final TunableNumber kP = new TunableNumber("PIDSetAngle kP", 0);
    private final TunableNumber kI = new TunableNumber("PIDSetAngle kI", 0);
    private final TunableNumber kD = new TunableNumber("PIDSetAngle kD", 0);

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

        // Clamp the speed
        pidRotation = MathUtils.clamp(pidRotation, -0.5, 0.5);

        drivetrain.arcadeDrive(0, pidRotation);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getYaw() - setpointAngle) < DriveConstants.ROTATION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}

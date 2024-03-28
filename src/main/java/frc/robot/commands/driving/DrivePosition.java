package frc.robot.commands.driving;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DrivePosition extends Command {
    private final Drivetrain drivetrain;
    private final Supplier<Translation2d> getTargetPosition;

    private final PIDController orientationController = new PIDController(0, 0, 0);
    private final PIDController translationController = new PIDController(0, 0, 0);

    private final Constraints orientationConstarints = new Constraints(0, 0);
    private final TrapezoidProfile orientationProfile = new TrapezoidProfile(orientationConstarints);

    private final Constraints translationConstarints = new Constraints(0, 0);
    private final TrapezoidProfile translationProfile = new TrapezoidProfile(translationConstarints);

    public DrivePosition(Drivetrain drivetrain, Supplier<Translation2d> getTargetPosition) {
        this.drivetrain = drivetrain;
        this.getTargetPosition = getTargetPosition;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d currentPosition = drivetrain.getPose().getTranslation();
        Translation2d targetPosition = getTargetPosition.get();
        double currentHeading = drivetrain.getYaw();
        double targetHeading = Math.atan2(targetPosition.getY() - currentPosition.getY(), targetPosition.getX() - currentPosition.getX());

        double translationSpeed = 0;
        double orientationSpeed = 0;

        // As long as the difference between the current and target heading
        // is greater than 5 degrees, we orient our drivetrain
        if (Math.abs(targetHeading - currentHeading) > Math.toRadians(5)) {
            State orientationSetpoint = orientationProfile.calculate(
                0.02,
                new State(drivetrain.getYaw(), 0),
                new State(targetHeading, 0)
            );
    
            orientationSpeed = orientationController.calculate(currentHeading, orientationSetpoint.position);
        } else {
            // Only move forward if we do not need to turn
            State translationSetpoint = translationProfile.calculate(
                0.02,
                getCurrentTranslationState(),
                new State(0, 0)
            );

            translationSpeed = translationController.calculate(translationSetpoint.position, 0);
        }

        drivetrain.arcadeDrive(translationSpeed, orientationSpeed);
    }

    /**
     * Calculate the distance between the drivetrain's current position
     * and the target position.
     */
    public double getTargetDistance() {
        Translation2d currentPosition = drivetrain.getPose().getTranslation();
        Translation2d targetPosition = getTargetPosition.get();

        return currentPosition.getDistance(targetPosition);
    }

    /**
     * @return The current translation and speed as a TrapezoidProfile State object
     */
    public State getCurrentTranslationState() {
        return new State(getTargetDistance(), drivetrain.getSpeeds().vxMetersPerSecond);
    }
}

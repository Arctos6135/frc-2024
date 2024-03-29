package frc.robot.commands.driving;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DrivePosition extends Command {
    private final Drivetrain drivetrain;
    private final Supplier<Translation2d> getTargetPosition;

    private final PIDController orientationController = new PIDController(5, 0, 0);
    private final PIDController translationController = new PIDController(0.1, 0, 0);

    private final Constraints orientationConstarints = new Constraints(1, 1);
    private final TrapezoidProfile orientationProfile = new TrapezoidProfile(orientationConstarints);

    private final Constraints translationConstarints = new Constraints(1, 1);
    private final TrapezoidProfile translationProfile = new TrapezoidProfile(translationConstarints);

    private double startTime;

    private State initialState;

    public DrivePosition(Drivetrain drivetrain, Supplier<Translation2d> getTargetPosition) {
        this.drivetrain = drivetrain;
        this.getTargetPosition = getTargetPosition;

        orientationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        Translation2d currentPosition = drivetrain.getPose().getTranslation();

        Translation2d targetPosition = getTargetPosition.get();

        Logger.recordOutput("DT/Target Position", targetPosition);
        double currentHeading = drivetrain.getPose().getRotation().getRadians();
        double targetHeading = targetPosition.minus(currentPosition).getAngle().getRadians();//Math.atan2(targetPosition.getY() - currentPosition.getY(), targetPosition.getX() - currentPosition.getX());

        Logger.recordOutput("DT/Yaw Target", targetHeading);

        double distance = targetPosition.getDistance(currentPosition);    

        initialState = new State(-distance, 0);
    }

    @Override
    public void execute() {
        Translation2d currentPosition = drivetrain.getPose().getTranslation();

        Translation2d targetPosition = getTargetPosition.get();

        Logger.recordOutput("DT/Target Position", targetPosition);
        double currentHeading = drivetrain.getPose().getRotation().getRadians();
        double targetHeading = targetPosition.minus(currentPosition).getAngle().getRadians();//Math.atan2(targetPosition.getY() - currentPosition.getY(), targetPosition.getX() - currentPosition.getX());

        Logger.recordOutput("DT/Yaw Target", targetHeading);

        double distance = targetPosition.getDistance(currentPosition);

        Logger.recordOutput("DT/Distance to Target", distance);

        Logger.recordOutput("DT/Orientation Control", orientationController.calculate(currentHeading, targetHeading));

        State translationSetpoint = translationProfile.calculate(
            Timer.getFPGATimestamp() - startTime,
            initialState,
            new State(0, 0)
        );

        Logger.recordOutput("DT/Velocity setpoint", translationSetpoint.velocity);

        Logger.recordOutput("DT/Position Setpoint", translationSetpoint.position);

        // translationController.calculate(0, translationSetpoint.position) + 
        drivetrain.arcadeDrive(translationSetpoint.velocity + translationController.calculate(-distance, 0), orientationController.calculate(currentHeading, targetHeading));


        // double translationSpeed = 0;
        // double orientationSpeed = 0;

        // // As long as the difference between the current and target heading
        // // is greater than 5 degrees, we orient our drivetrain
        // if (Math.abs(targetHeading - currentHeading) < Math.toRadians(20)) {
           
        //     // Only move forward if we do not need to turn
        //     State translationSetpoint = translationProfile.calculate(
        //         Timer.getFPGATimestamp() - startTime,
        //         getCurrentTranslationState(),
        //         new State(0, 0)
        //     );

        //     Logger.recordOutput("DrivePosition/translationSetpoint/position", translationSetpoint.position);
        //     Logger.recordOutput("DrivePosition/translationSetpoint/velocity", translationSetpoint.velocity);

        //     translationSpeed = translationController.calculate(currentPosition.getDistance(targetPosition), translationSetpoint.position) + translationSetpoint.velocity;
        // }

        // State orientationSetpoint = orientationProfile.calculate(
        //     Timer.getFPGATimestamp() - startTime,
        //     new State(drivetrain.getYaw(), drivetrain.getYawRate()),
        //     new State(targetHeading, 0)
        // );


        // Logger.recordOutput("DrivePosition/orientationSetpoint/position", orientationSetpoint.position);
        // Logger.recordOutput("DrivePosition/orientationSetpoint/velocity", orientationSetpoint.velocity);


        // orientationSpeed = orientationController.calculate(currentHeading, orientationSetpoint.position) + orientationSetpoint.velocity;

        // drivetrain.arcadeDrive(translationSpeed, orientationSpeed);

        // Logger.recordOutput("DrivePosition/currentPosition", currentPosition);
        // Logger.recordOutput("DrivePosition/targetPosition", targetPosition);
        Logger.recordOutput("DrivePosition/currentHeading", currentHeading);
        Logger.recordOutput("DrivePosition/targetHeading", targetHeading);
        // Logger.recordOutput("DrivePosition/currentPosition", currentPosition);
        // Logger.recordOutput("DrivePosition/translationSpeed", translationSpeed);
        // Logger.recordOutput("DrivePosition/orientationSpeed", orientationSpeed);
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

    @Override
    public boolean isFinished() {
        return drivetrain.getPose().getTranslation().getDistance(getTargetPosition.get()) < 0.05;
    }
}

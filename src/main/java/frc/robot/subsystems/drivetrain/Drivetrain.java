package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.characterization.AccelerationRoutine;
import frc.robot.commands.characterization.FeedforwardLog;
import frc.robot.commands.characterization.LoggedMechanism;
import frc.robot.commands.characterization.LoggedMechanismGroup;
import frc.robot.commands.characterization.Mechanism;
import frc.robot.commands.characterization.VelocityRoutine;

/**
 * A subsystem that controls the drivey bit of the robot.
 */
public class Drivetrain extends SubsystemBase {
    // The kind of drivetrain we are controlling: either simulation or real.
    private final DrivetrainIO io;
    // The current sensor readings for our drivetrain. This is actually a `DrivetrainIO.Inputs`, but the annotation on the class generated this lovely version for us that logs itself nicely.
    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    // PIDControllers that control the drivetrain motor voltage output
    private final PIDController leftController = new PIDController(0, 5, 0.0);
    private final PIDController rightController = new PIDController(0, 5, 0.0);

    // Simple feedforward controllers that determine how the drivetrain should behave
    private final SimpleMotorFeedforward leftForward = new SimpleMotorFeedforward(0.0, 2.213, 0.2);
    private final SimpleMotorFeedforward rightForward = new SimpleMotorFeedforward(0.0, 2.235, 0.26);

    // The target speed of the drivetrain. In m/s
    private double targetVelocityLeft = 0; 
    private double targetVelocityRight = 0;

    // The previous velocity of the drivetrain. Gets updated each loop. Used for feedforward. In m/s
    private double previousTargetVelocityLeft = 0;
    private double previousTargetVelocityRight = 0;

    private double leftAcceleration = 0;
    private double rightAcceleration = 0;


    /**
     * Construct a new odometry object.
     */
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromRadians(inputs.yaw), inputs.leftPosition, inputs.rightPosition);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.83));

    /**
     * Construct a new drivetrain.
     * @param io the kind of drivetrain we are controlling: either simulation or real
     */
    public Drivetrain(DrivetrainIO io) {
        this.io = io;

        AutoBuilder.configureRamsete(
            () -> odometry.getPoseMeters(), 
            (pose) -> {
                odometry.resetPosition(Rotation2d.fromRadians(inputs.yaw), new DifferentialDriveWheelPositions(inputs.leftPosition, inputs.rightPosition), pose);
            }, 
            () -> new ChassisSpeeds((inputs.leftVelocity + inputs.rightVelocity) / 2, 0, inputs.yawRate), 
            speeds -> arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond), 
            new ReplanningConfig(), 
            () -> false, 
            this
        );
    }

    @Override
    public void periodic() {
        // This tells our drivetrain-like-thing (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);
        // Log all the sensor data.
        Logger.processInputs("Drivetrain", inputs);
        // Get the rotation of the robot from the gyro.
        var gyroAngle = Rotation2d.fromRadians(inputs.yaw);
        // Update the odometry.
        var pose = odometry.update(gyroAngle, inputs.leftPosition, inputs.rightPosition);
        Logger.recordOutput("Odometry", pose);

        double leftVelocity = inputs.leftVelocity;
        double rightVelocity = inputs.rightVelocity;

        double leftFeedforwardEffort = leftForward.calculate(targetVelocityLeft, leftAcceleration);
        double leftFeedbackEffort = leftController.calculate(leftVelocity, targetVelocityLeft);

        double left = leftFeedforwardEffort + leftFeedbackEffort;

        double rightFeedforwardEffort = rightForward.calculate(targetVelocityRight, rightAcceleration);
        double rightFeedbackEffort = rightController.calculate(rightVelocity, targetVelocityRight);

        double right = rightFeedforwardEffort + rightFeedbackEffort;

        io.setVoltages(left, right);
    }

    /**
     * Set the target speed of the drivetrain.
     * @param throttle the forward speed of the drivetrain in m/s
     * @param turn the rotational speed of the drivetrain in rad/s
     */
    public void arcadeDrive(double throttle, double turn) {
        DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
        setSpeed(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
    }

    /**
     * Set the target speed of the drivetrain.
     * @param speedLeft the target speed of the left side in m/s
     * @param speedRight the target speed of the right side in m/s
     */
    public void setSpeed(double speedLeft, double speedRight) {
        Logger.recordOutput("Drivetrain Target Left Velocity", speedLeft);
        Logger.recordOutput("Drivetrain Target Right Velocity", speedRight);

        previousTargetVelocityLeft = targetVelocityLeft;
        previousTargetVelocityRight = targetVelocityRight;

        targetVelocityLeft = speedLeft;
        targetVelocityRight = speedRight;

        leftAcceleration = (targetVelocityLeft - previousTargetVelocityLeft) / 0.02;
        rightAcceleration = (targetVelocityRight - previousTargetVelocityRight) / 0.02;

        Logger.recordOutput("Drivetrain Target Left Acceleration", leftAcceleration);
        Logger.recordOutput("Drivetrain Target Right Acceleration", rightAcceleration);
    }

    /**
     * Get the average position of the two encoders. If the robot has only driven straight, this is the distance its travelled.
     */
    public double getDistance() {
        return (inputs.leftPosition + inputs.rightPosition) / 2;
    }

    public double getYaw() {
        return inputs.yaw;
    }

    // The drivetrain needs 3m of clearance in front of it when running this command.
    public Command characterizeVelocity() {
        FeedforwardLog leftLog = new FeedforwardLog();
        FeedforwardLog rightLog = new FeedforwardLog();

        Mechanism leftMechanism = new Mechanism(volts -> io.setVoltages(volts, volts), () -> inputs.leftPosition, () -> inputs.leftVelocity);
        Mechanism rightMechanism = new Mechanism(volts -> io.setVoltages(volts, volts), () -> inputs.rightPosition, () -> inputs.rightVelocity);

        LoggedMechanismGroup group = new LoggedMechanismGroup(
            new LoggedMechanism(leftLog, leftMechanism),
            new LoggedMechanism(rightLog, rightMechanism)
        );

        return new VelocityRoutine(group, 2.5, 0.25, this).finallyDo(() -> {
            leftLog.logCSV("DrivetrainVelocityLeft");
            rightLog.logCSV("DrivetrainVelocityRight");
        });
    }

    // The drivetrain needs 3m of clearance in front of it when running this command.
    public Command characterizeAcceleration() {
        FeedforwardLog leftLog = new FeedforwardLog();
        FeedforwardLog rightLog = new FeedforwardLog();

        Mechanism leftMechanism = new Mechanism(volts -> io.setVoltages(volts, volts), () -> inputs.leftPosition, () -> inputs.leftVelocity);
        Mechanism rightMechanism = new Mechanism(volts -> io.setVoltages(volts, volts), () -> inputs.rightPosition, () -> inputs.rightVelocity);

        LoggedMechanismGroup group = new LoggedMechanismGroup(
            new LoggedMechanism(leftLog, leftMechanism),
            new LoggedMechanism(rightLog, rightMechanism)
        );

        return new AccelerationRoutine(group, 2.5, 5, this).finallyDo(() -> {
            leftLog.logCSV("DrivetrainAccelerationLeft");
            rightLog.logCSV("DrivetrainAccelerationRight");
        });
    }
}
package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FeedforwardCharacterization;
import frc.robot.commands.FeedforwardCharacterization.Config;
import frc.robot.subsystems.Drivetrain.Drivetrain.DrivetrainIO;

/**
 * A subsystem that controls the drivey bit of the robot.
 */
public class Drivetrain extends SubsystemBase {
    // The kind of drivetrain we are controlling: either simulation or real.
    private final DrivetrainIO io;
    // The current sensor readings for our drivetrain. This is actually a `DrivetrainIO.Inputs`, but the annotation on the class generated this lovely version for us that logs itself nicely.
    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    // PIDControllers that control the drivetrain motor voltage output
    private final PIDController leftController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController rightController = new PIDController(0.0, 0.0, 0.0);

    // Simple feedforward controllers that determine how the drivetrain should behave
    private final SimpleMotorFeedforward leftForward = new SimpleMotorFeedforward(0.0, 2.2, 0.26);
    private final SimpleMotorFeedforward rightForward = new SimpleMotorFeedforward(0.0, 2.2, 0.26);


    // The target speed of the drivetrain. In m/s
    private double targetVelocityLeft = 0; 
    private double targetVelocityRight = 0;

    // The previous velocity of the drivetrain. Gets updated each loop. Used for feedforward. In m/s
    private double previousTargetVelocityLeft = 0;
    private double previousTargetVelocityRight = 0;

    private double leftAcceleration = 0;
    private double rightAcceleration = 0;

    /**
     * Construct a new drivetrain.
     * @param io the kind of drivetrain we are controlling: either simulation or real
     */
    public Drivetrain(DrivetrainIO io) {
        this.io = io;
    }

    /**
     * Construct a new odometry object.
     */
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromRadians(inputs.yaw), inputs.leftPosition, inputs.rightPosition);

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
     * Make the robot drive with the given translational power and rotational power.
     * @param translation the forward-backwards power in the range [-1, 1]
     * @param rotation the left-right turning power in the range [-1, 1]
     */
    public void arcadeDrive(double translation, double rotation) {
        io.setVoltages(12 * (translation + rotation), 12 * (translation - rotation));
    }

    /**
     * Set the target speed of the drivetrain.
     * @param speed the target speed in meters/sec
     */
    public void setSpeed(double speedLeft, double speedRight) {
        previousTargetVelocityLeft = targetVelocityLeft;
        previousTargetVelocityRight = targetVelocityRight;

        targetVelocityLeft = speedLeft;
        targetVelocityRight = speedRight;

        leftAcceleration = (targetVelocityLeft - previousTargetVelocityLeft) / 0.02;
        rightAcceleration = (targetVelocityRight - previousTargetVelocityRight) / 0.02;
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

    // The drivetrain needs 4m of clearance in front of and behind it when running this command.
    public Command characterize() {
        return new FeedforwardCharacterization(new Config(
            voltage -> {
                Logger.recordOutput("Feedforward Voltage", voltage);
                io.setVoltages(voltage, voltage);
            },
            this::getDistance,
            () -> (inputs.leftVelocity + inputs.rightVelocity) / 2, 
            6, 
            4, 
            2, 
            4, 
            "Drivetrain"
        ), this);
    }
}
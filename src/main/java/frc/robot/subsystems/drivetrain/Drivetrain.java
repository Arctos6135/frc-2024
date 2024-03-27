package frc.robot.subsystems.drivetrain;

import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import frc.robot.commands.characterization.AccelerationRoutine;
import frc.robot.commands.characterization.FeedforwardLog;
import frc.robot.commands.characterization.LoggedMechanism;
import frc.robot.commands.characterization.LoggedMechanismGroup;
import frc.robot.commands.characterization.Mechanism;
import frc.robot.commands.characterization.VelocityRoutine;
import frc.robot.util.MathUtils;
import frc.robot.util.TunableNumber;

/**
 * A subsystem that controls the drivey bit of the robot.
 */
public class Drivetrain extends SubsystemBase {
    // The kind of drivetrain we are controlling: either simulation or real.
    private final DrivetrainIO io;
    // The current sensor readings for our drivetrain. This is actually a `DrivetrainIO.Inputs`, but the annotation on the class generated this lovely version for us that logs itself nicely.
    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    // PIDControllers that control the drivetrain motor voltage output
    private final PIDController leftController = new PIDController(3, 0, 0.0);
    //private final PIDController leftController = new PIDController(0, 0, 0.0);
    private final PIDController rightController = new PIDController(12, 0, 0.0);
    //private final PIDController rightController = new PIDController(10, 0, 0.0);

    // Simple feedforward controllers that determine how the drivetrain should behave
    private final SimpleMotorFeedforward leftForward = new SimpleMotorFeedforward(0.228, 2.291, 0.473);
    private final SimpleMotorFeedforward rightForward = new SimpleMotorFeedforward(0.228, 2.351, 0.533);
    //private final SimpleMotorFeedforward leftForward = new SimpleMotorFeedforward(0.0, 2.2, 0.5);
    //private final SimpleMotorFeedforward rightForward = new SimpleMotorFeedforward(0.0, 2.2, 0.5);


    // The target speed of the drivetrain. In m/s
    private double targetVelocityLeft = 0; 
    private double targetVelocityRight = 0;

    // The previous velocity of the drivetrain. Gets updated each loop. Used for feedforward. In m/s
    private double previousTargetVelocityLeft = 0;
    private double previousTargetVelocityRight = 0;

    private double leftAcceleration = 0;
    private double rightAcceleration = 0;

    TunableNumber kPLeft = new TunableNumber("DT/Left/kP", 0.00001);
    TunableNumber kILeft = new TunableNumber("DT/Left/kI", 0);
    TunableNumber kDLeft = new TunableNumber("DT/Left/kD", 0);

    TunableNumber kPRight = new TunableNumber("DT/Right/kP", 0.00001);
    TunableNumber kIRight = new TunableNumber("DT/Right/kI", 0);
    TunableNumber kDRight = new TunableNumber("DT/Right/kD", 0);


    /**
     * Construct a new odometry object.
     */
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromRadians(inputs.yaw), inputs.leftPosition, inputs.rightPosition);

    public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.2));

    /**
     * Construct a new drivetrain.
     * @param io the kind of drivetrain we are controlling: either simulation or real
     */
    public Drivetrain(DrivetrainIO io) {
        this.io = io;

        io.configurePID(kPLeft.get(), kILeft.get(), kDLeft.get(), kPRight.get(), kIRight.get(), kDRight.get());

        AutoBuilder.configureLTV(
            this::getPose, 
            this::resetOdometry,
            () -> getSpeeds(), 
            speeds -> {
                arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);

                Logger.recordOutput("PathPlanner/command speeds", speeds);
            },
            new Vector<N3>(new SimpleMatrix(new double[] {0.0625, 0.125, 2})),
            new Vector<N2>(new SimpleMatrix(new double[] {1, 2})),
            0.02,
            new ReplanningConfig(true, true),
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, 
            this
        );
    }

    // @Override
    public void periodic() {
        io.updateInputs(inputs);

        // This tells our drivetrain-like-thing (either real or simulated) to update our class with all the sensor data.
        // Log all the sensor data.
        Logger.processInputs("Drivetrain", inputs);
        // Get the rotation of the robot from the gyro.
        var gyroAngle = Rotation2d.fromRadians(inputs.yaw);
        // Update the odometry.
        var pose = odometry.update(gyroAngle, inputs.leftPosition, inputs.rightPosition);
        Logger.recordOutput("Odometry", pose);


        leftAcceleration = (targetVelocityLeft - previousTargetVelocityLeft) / 0.02;
        rightAcceleration = (targetVelocityRight - previousTargetVelocityRight) / 0.02;

        leftAcceleration = MathUtils.clamp(leftAcceleration, -1, 1);
        rightAcceleration = MathUtils.clamp(rightAcceleration, -1, 1);

        previousTargetVelocityLeft = targetVelocityLeft;
        previousTargetVelocityRight = targetVelocityRight;

        Logger.recordOutput("DT/Left Acceleration Target", leftAcceleration);
        Logger.recordOutput("DT/Right Acceleration Target", rightAcceleration);
        

        double leftVelocity = inputs.leftVelocity;

        double leftFeedforwardEffort = leftForward.calculate(targetVelocityLeft, leftAcceleration);
        double leftFeedbackEffort = leftController.calculate(leftVelocity, targetVelocityLeft);

        double left =  leftFeedforwardEffort + leftFeedbackEffort;

        Logger.recordOutput("DT/Left Feedback", leftFeedbackEffort);
        Logger.recordOutput("DT/Left Feedforward", leftFeedforwardEffort);
        Logger.recordOutput("DT/Left Voltage", left);

        double rightVelocity = inputs.rightVelocity;

        double rightFeedforwardEffort = rightForward.calculate(targetVelocityRight, rightAcceleration);
        double rightFeedbackEffort = rightController.calculate(rightVelocity, targetVelocityRight);

        double right =  rightFeedforwardEffort + rightFeedbackEffort;

        Logger.recordOutput("DT/Right Feedback", rightFeedbackEffort);
        Logger.recordOutput("DT/Right Feedforward", rightFeedforwardEffort);
        Logger.recordOutput("DT/Right Voltage", right);

        //io.setVoltages((Math.abs(targetVelocityLeft) < 0.1) ? 0 : left, (Math.abs(targetVelocityRight) < 0.1) ? 0 : right);

        io.setSpeed(leftVelocity, rightVelocity, (Math.abs(targetVelocityLeft) < 0.1) ? 0 : leftFeedforwardEffort, (Math.abs(targetVelocityRight) < 0.1) ? 0  : rightFeedforwardEffort);
    
        TunableNumber.ifChanged(
            () -> {
                io.configurePID(kPLeft.get(), kILeft.get(), kDLeft.get(), kPRight.get(), kIRight.get(), kDRight.get());
            },
            kPLeft, kILeft, kDLeft, kPRight, kIRight, kDRight
        );
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
        Logger.recordOutput("DT/Left Velocity Target", speedLeft);
        Logger.recordOutput("DT/Right Velocity Target", speedRight);

        targetVelocityLeft = speedLeft;
        targetVelocityRight = speedRight;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(inputs.leftVelocity, inputs.rightVelocity));
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

    public double getYawRate() {
        return inputs.yawRate;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromRadians(inputs.yaw), new DifferentialDriveWheelPositions(inputs.leftPosition, inputs.rightPosition), pose);
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

        return new VelocityRoutine(group, 5, 0.25, this).finallyDo(() -> {
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

        return new AccelerationRoutine(group, 3.5, 5, this).finallyDo(() -> {
            leftLog.logCSV("DrivetrainAccelerationLeft");
            rightLog.logCSV("DrivetrainAccelerationRight");
        });
    }
}

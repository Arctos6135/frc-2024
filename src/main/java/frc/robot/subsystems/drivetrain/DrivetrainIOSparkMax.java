package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.constants.CANBus;
import frc.robot.constants.DriveConstants;
import frc.robot.util.TunableNumber;

/**
 * This class contains the very low level hardware details for the real robot's drivetrain.
 */
public class DrivetrainIOSparkMax extends DrivetrainIO {
    // Motor controllers that drive the robot.
    private final CANSparkMax rightMaster = new CANSparkMax(CANBus.DRIVE_RIGHT_MASTER, MotorType.kBrushless);
    private final CANSparkMax leftMaster = new CANSparkMax(CANBus.DRIVE_LEFT_MASTER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(CANBus.DRIVE_RIGHT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(CANBus.DRIVE_LEFT_FOLLOWER, MotorType.kBrushless);

    // Encoders to know what positions the wheels are at.
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final SparkPIDController leftController = leftMaster.getPIDController();
    private final SparkPIDController rightController = rightMaster.getPIDController();

    TunableNumber kPLeft = new TunableNumber("DT/Left/kP", 0.00001);
    TunableNumber kILeft = new TunableNumber("DT/Left/kI", 0);
    TunableNumber kDLeft = new TunableNumber("DT/Left/kD", 0);

    TunableNumber kPRight = new TunableNumber("DT/Right/kP", 0.00001);
    TunableNumber kIRight = new TunableNumber("DT/Right/kI", 0);
    TunableNumber kDRight = new TunableNumber("DT/Right/kD", 0);

    // Gyro.
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    public DrivetrainIOSparkMax() {
        // Set current limits to limit brownouts.
        rightMaster.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
        leftMaster.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
        rightFollower.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
        leftFollower.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);

        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);

        rightMaster.setInverted(true);
        leftMaster.setInverted(false);

        leftMaster.setIdleMode(IdleMode.kBrake);
        rightMaster.setIdleMode(IdleMode.kBrake);


        leftFollower.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

        leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);

        rightEncoder = rightMaster.getEncoder();
        leftEncoder = leftMaster.getEncoder();

        this.rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);
        this.leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);

        this.rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_FACTOR);
        this.leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_FACTOR);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        leftController.setP(kPLeft.get());
        leftController.setI(kILeft.get());
        leftController.setD(kDLeft.get());

        rightController.setP(kPLeft.get());
        rightController.setI(kILeft.get());
        rightController.setD(kDLeft.get());
    }   

    @Override
    public void setVoltages(double left, double right) {
        leftMaster.setVoltage(left);
        rightMaster.setVoltage(right);
    }

    @Override
    public void setSpeed(double left, double right, double leftFeedforward, double rightFeedforward) {
        leftController.setReference(left, ControlType.kVelocity, 0, leftFeedforward, ArbFFUnits.kVoltage);
        rightController.setReference(right, ControlType.kVelocity, 0, rightFeedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        inputs.leftPosition = leftEncoder.getPosition();
        inputs.rightPosition = rightEncoder.getPosition();

        inputs.leftVelocity = leftEncoder.getVelocity();
        inputs.rightVelocity = rightEncoder.getVelocity();

        inputs.yaw = Units.degreesToRadians(gyro.getAngle(gyro.getYawAxis()));
        inputs.yawRate = Units.degreesToRadians(gyro.getRate(gyro.getYawAxis()));

        // Current
        inputs.leftMasterCurrent = leftMaster.getOutputCurrent();
        inputs.rightMasterCurrent = rightMaster.getOutputCurrent();
        inputs.leftFollowerCurrent = leftMaster.getOutputCurrent();
        inputs.rightFollowerCurrent = rightMaster.getOutputCurrent();

        // Temperature
        inputs.leftMasterTemperature = leftMaster.getMotorTemperature();
        inputs.rightMasterTemperature = rightMaster.getMotorTemperature();
        inputs.leftFollowerTemperature = leftMaster.getMotorTemperature();
        inputs.rightFollowerTemperature = rightMaster.getMotorTemperature();

        // Voltage
        inputs.leftMasterVoltage = leftMaster.getBusVoltage() * leftMaster.getAppliedOutput();
        inputs.rightMasterVoltage = rightMaster.getBusVoltage() * rightMaster.getAppliedOutput();
        inputs.leftFollowerVoltage = leftMaster.getBusVoltage() * leftFollower.getAppliedOutput();
        inputs.rightFollowerVoltage = rightMaster.getBusVoltage() * rightFollower.getAppliedOutput();
    }
}

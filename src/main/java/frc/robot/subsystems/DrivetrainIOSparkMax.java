package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.DriveConstants;

/**
 * This class contains the very low level hardware details for the real robot's drivetrain.
 */
public class DrivetrainIOSparkMax extends DrivetrainIO {
    // Motor controllers that drive the robot.
    private final CANSparkMax rightMaster = new CANSparkMax(CANBus.RIGHT_MASTER, MotorType.kBrushless);
    private final CANSparkMax leftMaster = new CANSparkMax(CANBus.LEFT_MASTER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(CANBus.RIGHT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(CANBus.LEFT_FOLLOWER, MotorType.kBrushless);

    // Encoders to know what positions the wheels are at.
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    // Gyro.
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    public DrivetrainIOSparkMax() {
        this.rightFollower.follow(this.rightMaster);
        this.leftFollower.follow(this.leftMaster);

        this.rightMaster.setInverted(true);

        this.leftMaster.setIdleMode(IdleMode.kBrake);
        this.rightMaster.setIdleMode(IdleMode.kBrake);

        this.rightEncoder = this.rightMaster.getEncoder();
        this.leftEncoder = this.leftMaster.getEncoder();

        this.rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_FACTOR);
        this.leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_FACTOR);
    }

    @Override
    public void setVoltages(double left, double right) {
        leftMaster.set(left);
        rightMaster.set(right);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.leftPosition = leftEncoder.getPosition();
        inputs.rightPosition = rightEncoder.getPosition();

        inputs.leftVelocity = leftEncoder.getVelocity();
        inputs.rightVelocity = rightEncoder.getVelocity();

        inputs.yaw = gyro.getAngle(gyro.getYawAxis());
        inputs.yawRate = gyro.getRate(gyro.getYawAxis());
    }
}

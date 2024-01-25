package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ArmConstants;

public class ArmIOSparkMax extends ArmIO {
    // Motor that controls the arm.
    private final CANSparkMax armMotor = new CANSparkMax(CANBus.ARM_MOTOR, MotorType.kBrushless);

    // Encoder to know the arm's position.
    private final RelativeEncoder armEncoder;

    public ArmIOSparkMax() {
        this.armMotor.setIdleMode(IdleMode.kBrake);
        
        this.armEncoder = this.armMotor.getEncoder();

        this.armEncoder.setPositionConversionFactor(ArmConstants.ENCODER_CONVERSION_FACTOR);
    }

    @Override
    public void setVoltages(double voltage) {
        armMotor.set(voltage);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.position = armEncoder.getPosition();

        inputs.velocity = armEncoder.getVelocity();
    }
}
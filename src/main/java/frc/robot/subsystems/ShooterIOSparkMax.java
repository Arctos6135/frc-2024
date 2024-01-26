package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSparkMax extends ShooterIO {
    private final CANSparkMax right = new CANSparkMax(CANBus.SHOOTER_RIGHT, MotorType.kBrushless);
    private final CANSparkMax left = new CANSparkMax(CANBus.SHOOTER_LEFT, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    public ShooterIOSparkMax() {
        right.setInverted(true);

        right.setIdleMode(IdleMode.kBrake);
        left.setIdleMode(IdleMode.kBrake);

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_CONVERSION_FACTOR);
        leftEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_CONVERSION_FACTOR);
    }

    
    public void setVoltages(double rightVoltage, double leftVoltage) {
        right.set(rightVoltage);
        right.set(leftVoltage);
    }

    public void updateInputs(ShooterInputs inputs) {
        inputs.rightCurrent = right.getOutputCurrent();
        inputs.leftCurrent = left.getOutputCurrent();

        inputs.rightVelocity = rightEncoder.getVelocity();
        inputs.leftVelocity = leftEncoder.getVelocity();
    }
}
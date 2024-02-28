package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

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
        // Set current limits.
        right.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        left.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        
        right.setInverted(false);
        left.setInverted(true);

        right.setIdleMode(IdleMode.kBrake);
        left.setIdleMode(IdleMode.kBrake);

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightEncoder.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);
    
        rightEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    }
    
    public void setVoltages(double leftVoltage, double rightVoltage) {
        Logger.recordOutput("Shooter Voltage", leftVoltage);
        left.setVoltage(leftVoltage);
        right.setVoltage(rightVoltage);
    }

    public void updateInputs(ShooterInputs inputs) {
        inputs.rightVelocity = rightEncoder.getVelocity();
        inputs.leftVelocity = leftEncoder.getVelocity();

        // Current
        inputs.rightCurrent = right.getOutputCurrent();
        inputs.leftCurrent = left.getOutputCurrent();

        // Temperature
        inputs.leftTemperature = left.getMotorTemperature();
        inputs.rightTemperature = right.getMotorTemperature();

        // Voltage
        inputs.leftVoltage = left.getBusVoltage() * left.getAppliedOutput();
        inputs.rightVoltage = right.getBusVoltage() * right.getAppliedOutput();
    }
}
package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.constants.CANBus;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSparkMax extends ShooterIO {
    private final CANSparkMax right = new CANSparkMax(CANBus.SHOOTER_RIGHT, MotorType.kBrushless);
    private final CANSparkMax left = new CANSparkMax(CANBus.SHOOTER_LEFT, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final SparkPIDController PIDController = right.getPIDController();

    public ShooterIOSparkMax() {
        // Set current limits.
        right.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        left.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

        // Left motor is following the right one
        left.follow(right, true);

        right.setIdleMode(IdleMode.kBrake);
        left.setIdleMode(IdleMode.kBrake);

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightEncoder.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);
    
        rightEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    }
    
    public void setVoltages(double shooterVoltage) {
        Logger.recordOutput("Shooter Voltage", shooterVoltage);
        right.setVoltage(shooterVoltage);
    }

    public void setPIDTargetVelocity(double targetVelocity) {
        PIDController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public void calibratePIDController(double kP, double kI, double kD, double kFF) {
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setFF(kFF);
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
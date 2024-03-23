package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

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

    private final SparkPIDController rightPIDController = right.getPIDController();
    private final SparkPIDController leftPIDController = left.getPIDController();

    // TODO: calibrate Feedforward values
    private final SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(0, 0, 0);
    private final SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    public ShooterIOSparkMax() {
        // Set current limits.
        right.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        left.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

        // Left motor is following the right one
        // left.follow(right, true);

        left.setInverted(false);
        right.setInverted(true);

        right.setIdleMode(IdleMode.kBrake);
        left.setIdleMode(IdleMode.kBrake);

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightEncoder.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);
    
        rightEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    }
    
    public void setVoltage(double shooterVoltage) {
        Logger.recordOutput("Left Shooter Voltage", shooterVoltage);
        Logger.recordOutput("Right Shooter Voltage", shooterVoltage);
        right.setVoltage(shooterVoltage);
        left.setVoltage(shooterVoltage);
    }

    public void setVoltages(double leftShooterVoltage, double rightShooterVoltage) {
        Logger.recordOutput("Left Shooter Voltage", leftShooterVoltage);
        Logger.recordOutput("Right Shooter Voltage", rightShooterVoltage);
        right.setVoltage(rightShooterVoltage);
        left.setVoltage(leftShooterVoltage);
    }

    public void setPIDTargetVelocity(double targetVelocity) {
        rightPIDController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity, 0, rightFeedForward.calculate(targetVelocity));
        leftPIDController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity, 0, leftFeedForward.calculate(targetVelocity));
    }

    public void setPIDTargetVelocities(double leftTargetVelocity, double rightTargetVelocity) {
        rightPIDController.setReference(rightTargetVelocity, CANSparkMax.ControlType.kVelocity);
        leftPIDController.setReference(leftTargetVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public void calibratePIDController(double kP, double kI, double kD) {
        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);
        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);
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

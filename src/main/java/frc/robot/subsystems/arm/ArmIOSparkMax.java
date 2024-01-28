package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.ArmConstants;

public class ArmIOSparkMax extends ArmIO {
    // Motor that controls the arm.
    private final CANSparkMax armMotor = new CANSparkMax(CANBus.ARM_MOTOR, MotorType.kBrushless);

    // Encoder to know the arm's position.
    private final RelativeEncoder armEncoder;

    public ArmIOSparkMax() {
        // Sets current limit to prevent brownouts.
        armMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);

        armMotor.setIdleMode(IdleMode.kBrake);
        
        armEncoder = armMotor.getEncoder();

        armEncoder.setPositionConversionFactor(ArmConstants.ENCODER_CONVERSION_FACTOR);
        armEncoder.setVelocityConversionFactor(ArmConstants.ENCODER_CONVERSION_FACTOR);
        armEncoder.setPosition(ArmConstants.STARTING_POSITION);

        // sets up soft limits
        armMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, ArmConstants.MAX_POSITION); // need to make sure that max and min are in the right units and are the right value
        armMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, ArmConstants.MIN_POSITION);
    }

    @Override
    public void setVoltage(double voltage) {
        armMotor.set(voltage);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.position = armEncoder.getPosition();
        inputs.velocity = armEncoder.getVelocity();
    }
}
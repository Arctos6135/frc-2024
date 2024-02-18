package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.ArmConstants;

public class ArmIOSparkMax extends ArmIO {
    // Motors that control the arm.
    private final CANSparkMax armMaster = new CANSparkMax(CANBus.ARM_MASTER, MotorType.kBrushless);
    private final CANSparkMax armFollower = new CANSparkMax(CANBus.ARM_FOLLOWER, MotorType.kBrushless);

    // Encoder to know the arm's position.
    private final RelativeEncoder armEncoder;

    public ArmIOSparkMax() {
        armFollower.setInverted(true);

        armFollower.follow(armMaster);

        // Sets current limit to prevent brownouts.
        armMaster.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        armFollower.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);

        armMaster.setIdleMode(IdleMode.kBrake);
        
        armEncoder = armMaster.getEncoder();

        armEncoder.setPositionConversionFactor(ArmConstants.ENCODER_CONVERSION_FACTOR);
        armEncoder.setVelocityConversionFactor(ArmConstants.ENCODER_CONVERSION_FACTOR);
        armEncoder.setPosition(ArmConstants.STARTING_POSITION);

        // sets up soft limits
        armMaster.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        armMaster.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
        armMaster.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, ArmConstants.MAX_POSITION); // need to make sure that max and min are in the right units and are the right value
        armMaster.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, ArmConstants.MIN_POSITION);
    }

    @Override
    public void setVoltage(double voltage) {
        armMaster.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.position = armEncoder.getPosition();
        inputs.velocity = armEncoder.getVelocity();
    }
}
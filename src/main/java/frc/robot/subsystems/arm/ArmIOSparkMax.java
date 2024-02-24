package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANBus;

public class ArmIOSparkMax extends ArmIO {
    // Motors that control the arm.
    private final CANSparkMax armLeft = new CANSparkMax(CANBus.ARM_LEFT, MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(CANBus.ARM_RIGHT, MotorType.kBrushless);

    // Encoder to know the arm's position.
    private final RelativeEncoder armEncoder;

    public ArmIOSparkMax() {
        armLeft.setInverted(true);
        armRight.setInverted(false);


        // Sets current limit to prevent brownouts.
        armLeft.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        armRight.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);

        armLeft.setIdleMode(IdleMode.kCoast);
        armRight.setIdleMode(IdleMode.kCoast);
        
        armEncoder = armLeft.getEncoder();

        armEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
        armEncoder.setVelocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR);
        armEncoder.setPosition(ArmConstants.STARTING_POSITION);

        // // sets up soft limits
        // armLeft.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        // armLeft.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
        // armLeft.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, ArmConstants.MAX_POSITION); // need to make sure that max and min are in the right units and are the right value
        // armLeft.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, ArmConstants.MIN_POSITION);
 
        //  // sets up soft limits
        // armRight.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        // armRight.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
        // armRight.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, ArmConstants.MAX_POSITION); // need to make sure that max and min are in the right units and are the right value
       //  armRight.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, ArmConstants.MIN_POSITION);
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Arm Voltage", voltage);

        Logger.recordOutput("Left Motor Current", armLeft.getOutputCurrent());
        Logger.recordOutput("Right Motor Current", armRight.getOutputCurrent());

        Logger.recordOutput("Left Motor Power", armLeft.get());
        Logger.recordOutput("Right Motor Power", armRight.get());


        armLeft.setVoltage(voltage);
        armRight.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.position = armEncoder.getPosition();
        inputs.velocity = armEncoder.getVelocity();
    }
}
package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANBus;

public class ArmIOSparkMax extends ArmIO {
    // Motors that control the arm.
    private final CANSparkMax armLeft = new CANSparkMax(CANBus.ARM_LEFT, MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(CANBus.ARM_RIGHT, MotorType.kBrushless);

    // Encoder to know the arm's position.
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public ArmIOSparkMax() {
        armLeft.setInverted(false);
        armRight.setInverted(false);

        armLeft.follow(armRight, true);

        // Sets current limit to prevent brownouts.
        armLeft.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        armRight.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);

        armLeft.setIdleMode(IdleMode.kBrake);
        armRight.setIdleMode(IdleMode.kBrake);
        
        leftEncoder = armLeft.getEncoder();

        leftEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR);
        leftEncoder.setPosition(-ArmConstants.STARTING_POSITION);

        rightEncoder = armRight.getEncoder();

        rightEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setPosition(-ArmConstants.STARTING_POSITION);

        // sets up soft limits
        // we don't need tp enable soft stops on armLeft since it is following armRight
        armLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        armLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
 
         // sets up soft limits

        // armRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.MAX_POSITION); // need to make sure that max and min are in the right units and are the right value
        // armRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.MIN_POSITION);
        armRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        armRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

        // new Trigger(() -> new XboxController(0).getXButtonPressed()).onTrue(new InstantCommand(() -> {
        //     leftEncoder.setPosition(-ArmConstants.STARTING_POSITION);
        //     rightEncoder.setPosition(-ArmConstants.STARTING_POSITION);
        // }));
    }

    @Override
    public void setVoltage(double voltage) {
        armRight.setVoltage(-voltage);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {   
        // Position
        inputs.leftPosition = -leftEncoder.getPosition();
        inputs.rightPosition = rightEncoder.getPosition();

        // Velocity
        inputs.leftVelocity = -leftEncoder.getVelocity();
        inputs.rightPosition = rightEncoder.getVelocity();

        // Current
        inputs.leftCurrent = armLeft.getOutputCurrent();
        inputs.rightCurrent = armRight.getOutputCurrent();

        // Temperature
        inputs.leftTemperature = armLeft.getMotorTemperature();
        inputs.rightTemperature = armRight.getMotorTemperature();

        // Voltage
        inputs.leftVoltage = armLeft.getBusVoltage() * armLeft.getAppliedOutput();
        inputs.rightVoltage = armRight.getBusVoltage() * armRight.getAppliedOutput();
    }
}
package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSparkMax extends IntakeIO{
    private final CANSparkMax top = new CANSparkMax(CANBus.INTAKE_TOP_MASTER, MotorType.kBrushless);
    private final CANSparkMax bottom = new CANSparkMax(CANBus.INTAKE_BOTTOM_MASTER, MotorType.kBrushless);

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    public IntakeIOSparkMax() {
        top.setInverted(true);

        top.setIdleMode(IdleMode.kBrake);
        bottom.setIdleMode(IdleMode.kBrake);

        topEncoder = top.getEncoder();
        bottomEncoder = bottom.getEncoder();

        topEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
        bottomEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
    }

    
    public void setVoltages(double topVoltage, double bottomVoltage) {
        top.set(topVoltage);
        bottom.set(bottomVoltage);
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.topPosition = topEncoder.getPosition();
        inputs.bottomPosition = bottomEncoder.getPosition();

        inputs.topCurrent = top.getOutputCurrent();
        inputs.bottomCurrent = bottom.getOutputCurrent();
    }
}

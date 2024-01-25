package frc.robot.subsystems;

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
        this.top.setInverted(true);

        this.top.setIdleMode(IdleMode.kBrake);
        this.bottom.setIdleMode(IdleMode.kBrake);

        this.topEncoder = this.top.getEncoder();
        this.bottomEncoder = this.bottom.getEncoder();

        this.topEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
        this.bottomEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
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

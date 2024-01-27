package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSparkMax extends IntakeIO{
    private final CANSparkMax motor = new CANSparkMax(CANBus.INTAKE_TOP_MASTER, MotorType.kBrushless);

    private final RelativeEncoder topEncoder;

    public IntakeIOSparkMax() {
        motor.setInverted(true);

        motor.setIdleMode(IdleMode.kBrake);

        topEncoder = motor.getEncoder();

        topEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
    }

    
    public void setVoltage(double voltage) {
        motor.set(voltage);
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.topPosition = topEncoder.getPosition();

        inputs.topCurrent = motor.getOutputCurrent();
    }
}

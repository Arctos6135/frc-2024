package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSparkMax extends IntakeIO{
    private final CANSparkMax motor = new CANSparkMax(CANBus.INTAKE_TOP_MASTER, MotorType.kBrushless);

    private final RelativeEncoder encoder;

    public IntakeIOSparkMax() {
        motor.setInverted(true);

        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();

        encoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
    }

    
    public void setVoltage(double voltage) {
        motor.set(voltage);
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.position = encoder.getPosition();

        inputs.current = motor.getOutputCurrent();
    }
}

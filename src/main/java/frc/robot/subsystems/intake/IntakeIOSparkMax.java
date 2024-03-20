package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSparkMax extends IntakeIO{
    private final CANSparkMax motor = new CANSparkMax(CANBus.INTAKE_MOTOR, MotorType.kBrushless);

    private final RelativeEncoder encoder;

    public IntakeIOSparkMax() {
        // Sets current limits.
        motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

        motor.setInverted(true); // should this still be inverted

        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();

        // meters of belt
        encoder.setPositionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(IntakeConstants.VELOCITY_CONVERSION_FACTOR);
    }
    
    public void setVoltage(double voltage) {
        Logger.recordOutput("Intake/Voltage", voltage);
        motor.setVoltage(voltage);
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.position = encoder.getPosition();

        // Current
        inputs.current = motor.getOutputCurrent();

        // Temperature
        inputs.temperature = motor.getMotorTemperature();

        // Voltage
        inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();

        // Speed (don't need to convert because we do that by setting the velocity conversion factor).
        inputs.speed = encoder.getVelocity();
    }
}

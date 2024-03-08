package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;
import frc.robot.constants.WinchConstants;

public class WinchIOSparkMax extends WinchIO {
    private final CANSparkMax motor = new CANSparkMax(CANBus.WINCH_MOTOR, MotorType.kBrushless);

    private final RelativeEncoder encoder;

    public WinchIOSparkMax() {
        // Sets current limits.
        motor.setSmartCurrentLimit(WinchConstants.CURRENT_LIMIT);

        motor.setIdleMode(IdleMode.kCoast);

        encoder = motor.getEncoder();

        encoder.setPositionConversionFactor(WinchConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(WinchConstants.VELOCITY_CONVERSION_FACTOR);
    }

    public void setIdleMode(IdleMode idleMode) {
        motor.setIdleMode(idleMode);
    }

    public void setVoltage(double voltage) {
        Logger.recordOutput("Winch Voltage", voltage);
        motor.setVoltage(voltage);
    }
}

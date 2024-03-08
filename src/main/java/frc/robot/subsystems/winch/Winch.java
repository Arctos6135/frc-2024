package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;

public class Winch extends SubsystemBase {
    private final WinchIO io;

    private final WinchInputsAutoLogged inputs = new WinchInputsAutoLogged();

    public Winch(WinchIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // This tells our winch (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);
        // Log all the sensor data.
        Logger.processInputs("Winch", inputs);
    }

    /**
     * @return position in meters
     */
    public double getPosition() {
        return inputs.position;
    }

    public void setVoltage(double voltage) {
        Logger.recordOutput("Winch Voltage", voltage);
        io.setVoltage(voltage);
    }

    public void setIdleMode(IdleMode idleMode) {
        io.setIdleMode(idleMode);
    }
}

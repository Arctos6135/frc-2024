package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;


public class Arm extends SubsystemBase {
    private final ArmIO io;

    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // This tells our arm (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);
        // Log all the sensor data.
        Logger.processInputs("Arm", inputs);
    }

    public double getArmPosition() {
        return inputs.position;
    }

    public double getArmVelocity() {
        return inputs.velocity;
    }

    public void setVoltage(double voltage) {
        Logger.recordOutput("Arm Voltage", voltage);
        io.setVoltage(voltage);
    }
}
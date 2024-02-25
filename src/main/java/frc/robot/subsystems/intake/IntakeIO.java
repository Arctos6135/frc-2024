package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double position;

        // Amps
        public double current;

        // Celsius
        public double temperature;

        // % Max Volts
        public double voltage;

        // In meters of tread / second.
        public double speed;
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltage(double voltage) {}
}

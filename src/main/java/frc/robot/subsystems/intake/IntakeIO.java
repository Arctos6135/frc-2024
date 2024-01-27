package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double position;

        public double current; // assuming no resistance if their is conversion is needed
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltage(double voltage) {}
}

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double position;

        public double current;
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltage(double voltage) {}
}

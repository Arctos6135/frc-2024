package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double topPositionition;
        public double bottomPositionition;

        public double topCurrent; // assuming no resistance if their is conversion is needed
        public double bottomCurrent;
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltages(int topVoltage, int bottomVoltage) {}
}

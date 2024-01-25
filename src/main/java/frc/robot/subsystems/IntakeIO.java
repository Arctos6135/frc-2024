package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double topPosition;
        public double bottomPosition;

        public double topcurrent; // assuming no resistance if their is conversion is needed
        public double bottomCurrent;
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltage(double topVoltage, double bottomVoltage) {}
}

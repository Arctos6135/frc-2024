package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double topPos;
        public double bottomPos;

        public double topcurrent; // assuming no resistance if their is conversion is needed
        public double bottomCurrent;
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltage(int topVoltage, int bottomVoltage) {}
}

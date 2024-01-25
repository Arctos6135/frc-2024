package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double encoderPos;

        public double current; // assuming no resistance if their is conversion is needed
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setVoltage(int voltage) {}
}

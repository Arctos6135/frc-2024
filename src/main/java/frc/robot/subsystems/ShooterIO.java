package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        public double rightCurrent; // assuming no resistance if there is conversion needed
        public double leftCurrent;
    }

    public void updateInputs(ShooterInputs inputs) {}

    public void setVoltages(double rightVoltage, double leftVoltage) {}
}

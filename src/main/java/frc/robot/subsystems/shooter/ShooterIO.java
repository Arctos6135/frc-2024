package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        public double leftPosition;
        public double rightPosition;

        public double leftCurrent;
        public double rightCurrent;

        public double leftVelocity;
        public double rightVelocity;
    }

    public void updateInputs(ShooterInputs inputs) {}

    public void setVoltages(double leftVoltage, double rightVoltage) {}
}

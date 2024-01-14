package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public class DrivetrainIO {
    @AutoLog
    public static class Inputs {
        public double leftPosition;
        public double rightPosition;

        public double leftVelocity;
        public double rightVelocity;
        
        public double yaw;
        public double yawRate;
    }

    public void updateInputs(Inputs inputs) {}

    public void setVoltages(double left, double right) {}
}

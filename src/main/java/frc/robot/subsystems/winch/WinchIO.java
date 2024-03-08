package frc.robot.subsystems.winch;

import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.AutoLog;

public class WinchIO {
    @AutoLog
    public static class WinchInputs {
        public double position;
        
        public double current;

        public double temperature;
    }

    public void updateInputs(WinchInputs inputs) {}

    public void setVoltage(double voltage) {}

    public void setIdleMode(IdleMode idlemode) {}
}

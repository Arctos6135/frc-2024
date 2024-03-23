package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        public double leftPosition;
        public double rightPosition;

        public double leftVelocity;
        public double rightVelocity;

        // Amps
        public double leftCurrent; 
        public double rightCurrent;

        // Celsius
        public double leftTemperature;
        public double rightTemperature;

        // Voltage
        public double leftVoltage;
        public double rightVoltage;
    }

    public void updateInputs(ShooterInputs inputs) {}

    public void setVoltage(double shooterVoltage) {}

    public void setVoltages(double leftShooterVoltage, double rightShooterVoltage) {}

    public void setPIDTargetVelocity(double targetVelocity) {}

    public void setPIDTargetVelocities(double leftTargetVelocity, double rightTargetVelocity) {}

    public void calibratePIDController(double kP, double kI, double kD) {}
}

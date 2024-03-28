package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

/**
 * This class represents anything that behaves like a drivetrain. By channeling all our interactions with the drivetrain hardware through this class, 
 * we can have both the real robot and the simulated robot be "something that behaves like a drivetrain", allowing us to easily swap between them.
 */
public class DrivetrainIO {
    /**
     * This class represents all the sensor data (either real or simulated) of the drivetrain. Units are in meters, radians, and seconds.
     */
    @AutoLog
    public static class DrivetrainInputs {
        public double leftPosition;
        public double rightPosition;

        public double leftVelocity;
        public double rightVelocity;
        
        public double yaw;
        public double yawRate;

        // Amps
        public double rightMasterCurrent;
        public double leftMasterCurrent;
        public double rightFollowerCurrent;
        public double leftFollowerCurrent;

        // Celsius
        public double rightMasterTemperature;
        public double leftMasterTemperature;
        public double rightFollowerTemperature;
        public double leftFollowerTemperature;

        // % Max Volts
        public double rightMasterVoltage;
        public double leftMasterVoltage;
        public double rightFollowerVoltage;
        public double leftFollowerVoltage;
    }

    /**
     * Update the sensor data either using actual sensors or simulated values.
     * @param inputs the sensor data
     */
    public void updateInputs(DrivetrainInputs inputs) {}

    /**
     * Set the voltages of the left and right sides of the drivetrain.
     * @param left the voltage of the left side in the range [-12, 12]
     * @param right the voltage of the right side in the range [-12, 12]
     */
    public void setVoltages(double left, double right) {}

    public void setSpeed(double left, double right, double leftFeedforward, double rightFeedforward) {}

    public void configurePID(double kPLeft, double kILeft, double kDLeft, double kPRight, double kIRight, double kDRight) {}
}

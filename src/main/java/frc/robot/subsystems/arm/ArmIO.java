package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/**
 * This class represents anything that behaves like an arm. 
 */
public class ArmIO {
    /**
     * This class represents all the sensor data (either real or simulated) of the arm. Units are in meters, radians, and seconds.
     */
    @AutoLog
    public static class ArmInputs {
        // Zero radians should be parallel to the ground.
        public double position;

        // Radians per second
        public double velocity;
    }

    /**
     * Update the sensor data either using acutal sensors or simulated values.
     */
    public void updateInputs(ArmInputs inputs) {}

    /**
     * Set the voltage of the arm motor.
     * @param motor shh dont tell anyone 🤫
     */
    public void setVoltage(double motor) {}
}

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private final MedianFilter filter = new MedianFilter(ShooterConstants.MEDIAN_FILTER_SIZE);
    private double medianCurrent;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // This tells our Shooter (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);
        // Log all the sensor data.
        Logger.processInputs("Shooter", inputs);

        medianCurrent = filter.calculate(inputs.rightCurrent);
    }

    public void setVoltages(double leftVoltage, double rightVoltage) {
        io.setVoltages(leftVoltage, rightVoltage);
    }
    public double getFilteredCurrent() {
        return medianCurrent;
    }

}
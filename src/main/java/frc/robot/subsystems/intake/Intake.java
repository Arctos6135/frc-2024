package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;

    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private final MedianFilter filter = new MedianFilter(IntakeConstants.MEDIAN_FILTER_SIZE);
    private double medianCurrent;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // This tells our intake (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);
        // Log all the sensor data.
        Logger.processInputs("Intake", inputs);

        medianCurrent = filter.calculate(inputs.current);

        Logger.recordOutput("Intake Filtered Current", medianCurrent);
    }

    public double getVelocity() {
        return inputs.speed;
    }

    /**
     * @return position in meters
     */
    public double getPosition() {
        return inputs.position;
    }

    public void setVoltage(double voltage) {
        Logger.recordOutput("Intake Voltage", voltage);
        System.out.printf("Running at %s voltage\n", voltage);
        io.setVoltage(voltage);
    }

    public double getFilteredCurrent() {
        return medianCurrent;
    }
}

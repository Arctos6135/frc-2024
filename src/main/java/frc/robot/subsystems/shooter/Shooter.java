package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA
    );

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // This tells our Shooter (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);
        // Log all the sensor data.
        Logger.processInputs("Shooter", inputs);
    }

    public void setVoltages(double leftVoltage, double rightVoltage) {
        Logger.recordOutput("Shoot Left Voltage", leftVoltage);
        Logger.recordOutput("Shoot Right Voltage", rightVoltage);
        io.setVoltages(leftVoltage, rightVoltage);
    }

    public void setRPS(double rps) {
        double feedforwardOutput = feedforward.calculate(rps);
        Logger.recordOutput("Shoot Velocity Target", rps);

        double leftOutput = feedforwardOutput;
        double rightOutput = feedforwardOutput;

        setVoltages(leftOutput, rightOutput);
    }

    public void stop() {
        setVoltages(0, 0);
    }

    public double getVelocity() {
        return (inputs.leftVelocity + inputs.rightVelocity) / 2;
    }
    
    public double getPosition() {
        return inputs.leftPosition;
    }
}
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

    public void setVoltage(double voltage) {
        Logger.recordOutput("Shooter Left Voltage", voltage);
        Logger.recordOutput("Shooter Right Voltage", voltage);
        io.setVoltage(voltage);
    }

    public void setVoltages(double leftVoltage, double rightVoltage) {
        io.setVoltages(leftVoltage, rightVoltage);
    }

    public void setRPS(double rps) {
        double feedforwardOutput = feedforward.calculate(rps);
        Logger.recordOutput("Shoot Velocity Target", rps);

        setVoltage(feedforwardOutput);
    }

    public void stop() {
        setVoltage(0);
    }

    public void calibratePIDController(double kP, double kI, double kD, double kFF) {
        io.calibratePIDController(kP, kI, kD, kFF);
    }

    public void setPIDTargetVelocity(double targetVelocity) {
        io.setPIDTargetVelocity(targetVelocity);
    }

    public void setPIDTargetVelocities(double leftTargetVelocity, double rightTargetVelocity) {
        io.setPIDTargetVelocities(leftTargetVelocity, rightTargetVelocity);
    }

    public double getVelocity() {
        return (inputs.leftVelocity + inputs.rightVelocity) / 2;
    }
    
    public double getPosition() {
        return inputs.leftPosition;
    }

    public double getCurrent() {
        return (inputs.leftCurrent + inputs.rightCurrent) / 2;
    }

    public double getVoltage(){
        return (inputs.leftVoltage + inputs.rightVoltage) / 2;
    }
}

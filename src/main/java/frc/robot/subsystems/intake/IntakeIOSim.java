package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;

import com.revrobotics.CANSparkMax;


// These constants need to be tuned and do not accurately represent the intake.
public class IntakeIOSim extends IntakeIO {
    private final FlywheelSim intake = new FlywheelSim(DCMotor.getNeo550(1), 5, IntakeConstants.MOMENT_OF_INERTIA);
    private double voltage;

     @Override
    public void setVoltage(double voltage) {
        intake.setInputVoltage(voltage);
        this.voltage = voltage;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        intake.update(0.2);

        inputs.speed = intake.getAngularVelocityRadPerSec() * IntakeConstants.WHEEL_CIRCUMFERENCE / (2 * Math.PI);
        inputs.voltage = this.voltage;
        inputs.current = intake.getCurrentDrawAmps();
    }
}
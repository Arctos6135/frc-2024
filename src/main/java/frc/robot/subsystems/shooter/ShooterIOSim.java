package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class ShooterIOSim extends ShooterIO {
    // private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA);
    
    private final FlywheelSim left = new FlywheelSim(DCMotor.getNEO(1), 1, ShooterConstants.MOMENT_OF_INERTIA);
    private final FlywheelSim right = new FlywheelSim(DCMotor.getNEO(1), 1, ShooterConstants.MOMENT_OF_INERTIA);

    @Override
    public void setVoltage(double shooterVoltage) {
        left.setInputVoltage(shooterVoltage);
        right.setInputVoltage(shooterVoltage);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        left.update(0.2);
        right.update(0.2);

        inputs.leftVelocity = left.getAngularVelocityRPM() / 60;
        inputs.rightVelocity = right.getAngularVelocityRPM() / 60;
    }

 
    public void setPIDTargetVelocity(double targetVelocity) {
        io.setPIDTargetVelocity(targetVelocity);
    }

    public void setPIDTargetVelocities(double leftTargetVelocity, double rightTargetVelocity) {
        io.setPIDTargetVelocities(leftTargetVelocity, rightTargetVelocity);
    }

    public void calibratePIDController(double kP, double kI, double kD, double kFF) {
        io.calibratePIDController(kP, kI, kD, kFF);
    }

  
}

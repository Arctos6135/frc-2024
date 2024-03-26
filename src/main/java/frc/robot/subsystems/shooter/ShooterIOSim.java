package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class ShooterIOSim extends ShooterIO {
    // private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA);
    
    private final FlywheelSim left = new FlywheelSim(DCMotor.getNEO(1), 1, ShooterConstants.MOMENT_OF_INERTIA);
    private final FlywheelSim right = new FlywheelSim(DCMotor.getNEO(1), 1, ShooterConstants.MOMENT_OF_INERTIA);

    private final PIDController leftPIDController = new PIDController(0, 0, 0);
    private final PIDController rightPIDController = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(0, 0, 0);
    private final SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    // private double leftTargetVelocity;
    // private double rightTargetVelocity;

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

        left.setInputVoltage(leftPIDController.calculate(inputs.leftVelocity) + leftFeedForward.calculate(inputs.leftVelocity));
        right.setInputVoltage(rightPIDController.calculate(inputs.leftVelocity) + rightFeedForward.calculate(inputs.leftVelocity));
    }



    public void setPIDTargetVelocities(double leftTargetVelocity, double rightTargetVelocity) {
        leftPIDController.setSetpoint(leftTargetVelocity);
        rightPIDController.setSetpoint(rightTargetVelocity);
    }

    public void setPIDTargetVelocity(double targetVelocity) {
        leftPIDController.setSetpoint(targetVelocity);
        rightPIDController.setSetpoint(targetVelocity);
    }

    public void calibratePIDController(double kP, double kI, double kD) {
        leftPIDController.setPID(kP, kP, kD);
        rightPIDController.setPID(kP, kI, kD);
    }

  
}

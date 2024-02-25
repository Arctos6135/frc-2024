package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.ShooterConstants;


public class ShooterIOSim extends ShooterIO {
    private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA);
    
    private final FlywheelSim left = new FlywheelSim(flywheelPlant, DCMotor.getNEO(1), 1);
    private final FlywheelSim right = new FlywheelSim(flywheelPlant, DCMotor.getNEO(1), 1);

    @Override
    public void setVoltages(double leftVoltage, double rightVoltage) {
        left.setInputVoltage(leftVoltage);
        right.setInputVoltage(rightVoltage);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        left.update(0.2);
        right.update(0.2);

        inputs.leftVelocity = left.getAngularVelocityRPM() / 60;
        inputs.rightVelocity = right.getAngularVelocityRPM() / 60;
    }
}
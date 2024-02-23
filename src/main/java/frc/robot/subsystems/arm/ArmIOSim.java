package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;

public class ArmIOSim extends ArmIO {
    private final double MOMENT_OF_INERTIA = SingleJointedArmSim.estimateMOI(
        ArmConstants.ARM_LENGTH, 
        ArmConstants.ARM_WEIGHT
    );

    private final SingleJointedArmSim arm = new SingleJointedArmSim(
        DCMotor.getNEO(2),
        1.0 / ArmConstants.GEARBOX_RATIO,
        MOMENT_OF_INERTIA,
        ArmConstants.ARM_LENGTH,
        ArmConstants.MIN_POSITION,
        ArmConstants.MAX_POSITION,
        true,
        ArmConstants.STARTING_POSITION
    );

    @Override
    public void updateInputs(ArmInputs inputs) {
        arm.update(0.02);

        inputs.position = arm.getAngleRads();
        inputs.velocity = arm.getVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double voltage) {
        arm.setInputVoltage(voltage);
    }
}

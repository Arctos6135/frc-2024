package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/**
 * This class simulates a drivetrain.
 */
public class DrivetrainIOSim extends DrivetrainIO {
    private final DifferentialDrivetrainSim drive = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch, null
    );

    private double previousHeading = 0;

    @Override
    public void updateInputs(Inputs inputs) {
        drive.update(0.02);

        inputs.leftPosition = drive.getLeftPositionMeters();
        inputs.rightPosition = drive.getRightPositionMeters();

        inputs.leftVelocity = drive.getLeftVelocityMetersPerSecond();
        inputs.rightVelocity = drive.getRightVelocityMetersPerSecond();

        inputs.yaw = drive.getHeading().getRadians();
        inputs.yawRate = (inputs.yaw - previousHeading) / 0.02;
        previousHeading = inputs.yaw;

        Logger.recordOutput("Drivetrain Sim Pose", drive.getPose());
    }

    @Override
    public void setVoltages(double left, double right) {
        drive.setInputs(left, right);
    }
}

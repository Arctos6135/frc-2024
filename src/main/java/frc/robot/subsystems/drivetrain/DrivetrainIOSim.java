package frc.robot.subsystems.drivetrain;

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
    public void updateInputs(DrivetrainInputs inputs) {
        drive.update(0.02);

        inputs.leftPosition = drive.getLeftPositionMeters();
        inputs.rightPosition = drive.getRightPositionMeters();

        inputs.leftVelocity = drive.getLeftVelocityMetersPerSecond();
        inputs.rightVelocity = drive.getRightVelocityMetersPerSecond();

        inputs.yaw = drive.getHeading().getRadians();
        inputs.yawRate = (inputs.yaw - previousHeading) / 0.02;
        previousHeading = inputs.yaw;

        Logger.recordOutput("Drivetrain Sim/Pose", drive.getPose());//derek was here
    }

    @Override
    public void setVoltages(double left, double right) {
        // if (Math.abs(left) < 0.5) {
        //     left = 0;
        // } else {
        //     left -= 0.2;
        // }

        // if (Math.abs(right) < 0.5) {
        //     right = 0;
        // } else {
        //     right -= 0.2;
        // }
        // drive.setInputs(left + Math.random() - 0.5, right + Math.random() - 0.5);

        drive.setInputs(left, right);
    }
}

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
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


    private final PIDController leftController = new PIDController(2, 0, 0.0);
    private final PIDController rightController = new PIDController(2, 0, 0.0);

    private double leftFeedforward = 0;
    private double rightFeedforward = 0;

    private double leftTarget = 0;
    private double rightTarget = 0;

    private double previousHeading = 0;

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        setVoltages(
            leftFeedforward * (2 - 0.3) + leftController.calculate(drive.getLeftVelocityMetersPerSecond()), 
            rightFeedforward * 1.7 + rightController.calculate(drive.getRightVelocityMetersPerSecond())
        );

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
        drive.setInputs(left, right);
    }

    @Override
    public void setSpeed(double left, double right, double leftFeedforward, double rightFeedforward) {
        this.leftFeedforward = leftFeedforward;
        this.rightFeedforward = rightFeedforward;

        leftTarget = left;
        rightTarget = right;
    }
}

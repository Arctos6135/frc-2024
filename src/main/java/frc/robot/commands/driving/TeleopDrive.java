package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Dampener;

public class TeleopDrive extends Command {
    private final Drivetrain drivetrain;

    private final XboxController controller;

    private Dampener xDampener = new Dampener(0.5);
    private Dampener yDampener = new Dampener(0.5);

    private boolean precisionDrive = false;

    public TeleopDrive(Drivetrain drivetrain, XboxController driverController) {
        this.drivetrain = drivetrain;
        this.controller = driverController;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double y = controller.getRawAxis(XboxController.Axis.kLeftY.value);
        // Rounds y to zero if it's less than the controller's deadband.
        if (Math.abs(y) < ControllerConstants.CONTROLLER_DEADZONE) {y = 0.0;}
        double y1 = -yDampener.dampen(y) * (precisionDrive ? DriveConstants.PRECISION_FWD_REV : 1.0);
        double x = controller.getRawAxis(XboxController.Axis.kRightX.value);
        // Rounds x to zero if it's less than the controller's deadband.
        if (Math.abs(x) < ControllerConstants.CONTROLLER_DEADZONE) {x = 0.0;}
        double x1 = xDampener.dampen(x) * (precisionDrive ? DriveConstants.PRECISION_TURN : 1.0);

        y1 *= DriveConstants.MAX_TELEOP_SPEED;
        x1 *= DriveConstants.MAX_TURN_SPEED_FACTOR;

        drivetrain.setSpeed(y1 + x1, y1 - x1);
    }

    public void setPrecisionDrive(boolean precise) {
        precisionDrive = precise;
    }
}

package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Dampener;

public class TeleopDrive extends Command {
    private final Drivetrain drivetrain;

    private final XboxController controller;

    private final int X_AXIS;
    private final int Y_AXIS;

    private Dampener xDampener;
    private Dampener yDampener;

    private static boolean precisionDrive = false;

    public TeleopDrive(Drivetrain drivetrain, XboxController driverController, int fwdRevAxis, int leftRightAxis){
        this.drivetrain = drivetrain;
        this.controller = driverController;

        this.X_AXIS = leftRightAxis;
        this.Y_AXIS = fwdRevAxis;

        this.xDampener = new Dampener(0.5);
        this.yDampener = new Dampener(0.5);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        double y = controller.getRawAxis(XboxController.Axis.kLeftY.value);
        double y1 = -yDampener.dampen(y) * (precisionDrive ? DriveConstants.PRECISION_FWD_REV : 1.0);
        double x = controller.getRawAxis(XboxController.Axis.kRightX.value);
        double x1 = xDampener.dampen(x) * (precisionDrive ? DriveConstants.PRECISION_TURN : 1.0);

        drivetrain.arcadeDrive(y1 * 0.85, x1 * 0.25);
    }

    public static boolean isPrecisionDrive() {
        return precisionDrive;
    }

    public static void setPrecisionDrive(boolean precisionDrive) {
        TeleopDrive.precisionDrive = precisionDrive;
    }

    public static Command togglePrecisionDrive() {
        return new InstantCommand(() -> {
            precisionDrive = !precisionDrive;
        });
    }
}

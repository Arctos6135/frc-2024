package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Dampener;
import frc.robot.constants.DriveConstants;

public class TeleopDrive {
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

        this.xDampener = new Dampener(DriveConstants.CONTROLLER_DEADZONE, 6);
        this.yDampener = new Dampener(DriveConstants.CONTROLLER_DEADZONE, 4);
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        double y = controller.getRawAxis(XboxController.Axis.kLeftY.value);
        double y1 = -yDampener.dampen(y) * (precisionDrive ? DriveConstants.precisionFwdRev : 1.0);
        double x = controller.getRawAxis(XboxController.Axis.kRightX.value);
        double x1 = xDampener.dampen(x) * (precisionDrive ? DriveConstants.precisionTurn : 1.0);

        SmartDashboard.putNumber("drive controller x", x1);
        SmartDashboard.putNumber("drive controller y", y1);
        
        double precisionFwdFactor = precisionDrive ? DriveConstants.precisionFwdRev : 1;
        double precisionTurnFactor = precisionDrive ? DriveConstants.precisionTurn : 1;

        drivetrain.arcadeDrive(y1 * 0.85 * precisionFwdFactor, x1 * 0.25 * precisionTurnFactor);
    }

    public static boolean isPrecisionDrive() {
        return precisionDrive;
    }

    public static void setPrecisionDrive(boolean precisionDrive) {
        TeleopDrive.precisionDrive = precisionDrive;
    }

    public static Command togglePrecisionDrive() {
        precisionDrive = !precisionDrive;
        return null;
    }

}

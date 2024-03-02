package frc.robot.commands.driving;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ProfiledPIDSetAngle extends ProfiledPIDCommand {
    private final double yawGoal;
    private final Drivetrain drive;
    public ProfiledPIDSetAngle(Drivetrain drive, double yaw) {
        super(
            new ProfiledPIDController(24, 0, 0.2, new Constraints(3, 8)), 
            drive::getYaw, 
            new TrapezoidProfile.State(yaw, 0), 
            (output, goal) -> {
                drive.arcadeDrive(0, output);
                Logger.recordOutput("Drivetrain/Goal Yaw", goal.position);
            }, 
            drive
        );
        yawGoal = yaw;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        super.initialize();

        Logger.recordOutput("Drivetrain/Setting Angle", true);
        Logger.recordOutput("Drivetrain/Target Yaw", yawGoal);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getYawRate()) < DriveConstants.ROTATION_TOLERANCE && Math.abs(drive.getYaw() - yawGoal) < DriveConstants.ROTATION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        Logger.recordOutput("Drivetrain/Setting Angle", false);
    }
}

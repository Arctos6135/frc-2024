package frc.robot.commands.driving;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveForward extends Command {
    private final Drivetrain drivetrain;

    private final double speed;
    private final double distance;
    private double initialDistance;

    // TODO: use PID instead of a Band Bang controller
    public DriveForward(Drivetrain drivetrain, double speed, double distance) {
        this.speed = Math.copySign(Math.abs(speed), distance);
        this.drivetrain = drivetrain;
        this.distance = Math.max(0, Math.abs(distance) - 14); // as long as the robot reaches full speed, it fairly consistently overshoots by ~14 inches
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        initialDistance = drivetrain.getDistance();
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(speed, 0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(drivetrain.getDistance() - initialDistance) > this.distance) {
            System.out.printf("finished driving %f meters\n", this.distance);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final DrivetrainIO io;

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
    }

    public void arcadeDrive(double translation, double rotation) {
        io.setVoltages(12 * (translation + rotation), 12 * (translation - rotation));
    }
}
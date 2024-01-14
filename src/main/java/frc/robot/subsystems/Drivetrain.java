package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final DrivetrainIO io;
    private final InputsAutoLogged inputs = new InputsAutoLogged();

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);
    }

    public void arcadeDrive(double translation, double rotation) {
        io.setVoltages(12 * (translation + rotation), 12 * (translation - rotation));
    }
}
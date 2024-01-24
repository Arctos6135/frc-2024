package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FeedforwardCharacterization;
import frc.robot.commands.FeedforwardCharacterization.Config;

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

    public double getDistance() {
        return (inputs.leftPosition + inputs.rightPosition) / 2;
    }

    public double getYaw() {
        return inputs.yaw;
    }

    // The drivetrain needs 4m of clearance in front of and behind it when running this command.
    public Command characterize() {
        return new FeedforwardCharacterization(new Config(
            voltage -> io.setVoltages(voltage, voltage),
            this::getDistance,
            () -> (inputs.leftVelocity + inputs.rightVelocity) / 2, 
            6, 
            12, 
            2, 
            4, 
            "Drivetrain"
        ));
    }
}
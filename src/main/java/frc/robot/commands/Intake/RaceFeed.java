package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RaceFeed extends Command {
    Shooter shooter;
    Intake intake;

    public RaceFeed(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void execute() {
        shooter.setVoltage(0.8);
        intake.setVoltage(12);
    }

    @Override
    public void end(boolean i) {
        shooter.setVoltage(0);

        intake.setVoltage(0);
    }
}

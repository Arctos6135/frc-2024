package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoIntake extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public AutoIntake(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        intake.setVoltage(12);
        shooter.setVoltage(-12);
    }

    @Override
    public void end(boolean i) {
        intake.setVoltage(0);
        shooter.setVoltage(0);
    }
}

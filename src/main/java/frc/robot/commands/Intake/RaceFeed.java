package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TunableNumber;

public class RaceFeed extends Command {
    Shooter shooter;
    Intake intake;
    TunableNumber voltage = new TunableNumber("Voltage", 12);

    public RaceFeed(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void execute() {
        shooter.setVoltage(0.8);
        intake.setVoltage(voltage.get());
    }

    @Override
    public void end(boolean i) {
        shooter.setVoltage(0);

        intake.setVoltage(0);
    }
}

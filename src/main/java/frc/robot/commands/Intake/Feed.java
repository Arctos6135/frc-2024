package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.TunableNumber;

public class Feed extends Command {
    private final Intake intake;
    TunableNumber voltage = new TunableNumber("Voltage", 12);

    public Feed(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setVoltage(voltage.get());
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }
}

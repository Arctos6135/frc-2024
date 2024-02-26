package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.shooter.AdvanceShooter;

public class FeedToShooter extends Command{
    private final Intake intake;
    private final Shooter shooter;
    // private final AdvanceShooter advanceShooter;
    private double initialShooterVoltage = 0;

    public FeedToShooter(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        // this.advanceShooter = advanceShooter;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
        shooter.setRPS(ShooterConstants.FEED_RPS);
        initialShooterVoltage = shooter.getVoltage();
    }

    @Override
    public void execute() {
        if (shooter.getVoltage() != initialShooterVoltage) {
            double distance = Units.inchesToMeters(15) / (ShooterConstants.WHEEL_DIAMETER / 2)
            new AdvanceShooter(shooter, distance);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getFilteredCurrent() <= IntakeConstants.INTAKE_CURRENT;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
        shooter.setRPS(0);
    }
}


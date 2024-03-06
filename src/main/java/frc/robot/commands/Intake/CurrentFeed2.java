package frc.robot.commands.Intake;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.AdvanceShooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CurrentFeed2 extends Command{
    private final Intake intake;
    private final Shooter shooter;
    private double maxCurrent;
    private final AdvanceIntake advanceIntake;
    private final AdvanceShooter advanceShooter;
    private boolean touchedShooter;
    private double shooterCurrent;
    

    private double startTime;

    public CurrentFeed2(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        maxCurrent = 0;
        this.advanceIntake = new AdvanceIntake(intake, Units.inchesToMeters(12));
        this.advanceShooter = new AdvanceShooter(shooter, ShooterConstants.ADVANCE_DISTANCE);
        maxCurrent = 0;
        touchedShooter = false;
        shooterCurrent = 0;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
        shooter.setVoltages(ShooterConstants.FEED_VOLTAGE);
        startTime = Timer.getFPGATimestamp();
        shooterCurrent = shooter.getCurrent();
    }

    @Override
    public void execute() {
        double current = intake.getFilteredCurrent();
        if (current > maxCurrent) {
            maxCurrent = current;
        }

        if (Math.abs(shooterCurrent - shooter.getCurrent()) > 0.1) {
            touchedShooter = true;
            // advanceShooter.raceWith(advanceIntake).schedule();
            advanceShooter.schedule();
            advanceIntake.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return (maxCurrent - intake.getFilteredCurrent() >= 20 || advanceShooter.isFinished() || advanceIntake.isFinished()) && (Timer.getFPGATimestamp() - startTime) > 0.25;
        // return intake.getFilteredCurrent() <= 8 && (Timer.getFPGATimestamp() - startTime) > 0.25;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
        shooter.stop();
    }
}

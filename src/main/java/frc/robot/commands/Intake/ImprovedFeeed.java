import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.AdvanceShooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ImprovedFeeed extends Command{
    private final Intake intake;
    private final Shooter shooter;
    private final ReversePIDFeed reversePIDFeed;
    private final AdvanceShooter advanceShooter;
    private double maxCurrent;
    private double startTime;
    private boolean peaked;
    private boolean touchedShooter;
    private double shooterCurrent;
    private boolean jankFix = true;

    public ImprovedFeeed(Intake intake, Shooter shooter, ReversePIDFeed reversePIDFeed, AdvanceShooter advanceShooter) {
        this.intake = intake;
        this.shooter = shooter;
        this.reversePIDFeed = reversePIDFeed;
        this.advanceShooter = advanceShooter;
        maxCurrent = 0;
        peaked = false;
        touchedShooter = false;
        shooterCurrent = 0;


        addRequirements(intake, shooter); // might want to remove If I understand this correctly
    }

    @Override 
    public void initialize() {
        intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double current = intake.getFilteredCurrent();
        if (current < maxCurrent && !peaked) {
            peaked = true;
            reversePIDFeed.schedule();
        }

        if (reversePIDFeed.isFinished() && !touchedShooter) {
            if (jankFix) {
                intake.setVoltage(IntakeConstants.FEED_VOLTAGE);
                shooter.setRPS(ShooterConstants.FEED_RPS);
                shooterCurrent = shooter.getCurrent();
            }
            else {
                double tempCurrent = shooter.getCurrent();
                if (tempCurrent < shooterCurrent) {
                    touchedShooter = true;
                    advanceShooter.schedule();
                }
            }    
        }
    }

    @Override
    public boolean isFinished() {
        return touchedShooter && advanceShooter.isFinished();
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
        shooter.stop();
    }
}

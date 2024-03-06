package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.AdvanceShooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AltImprovedFeed extends Command{
    private final Intake intake;
    private final Shooter shooter;
    private final AdvanceIntake reverseIntake;
    private final AdvanceIntake advanceIntake;
    private final AdvanceShooter advanceShooter;
    private double startTime;
    private double maxCurrent;
    private boolean peaked;
    private boolean touchedShooter;
    private double shooterCurrent;
    private boolean jankFix = true;

    public AltImprovedFeed(Intake intake, Shooter shooter, AdvanceIntake advanceIntake, AdvanceIntake reverseIntake, AdvanceShooter advanceShooter) {
        this.intake = intake;
        this.shooter = shooter;
        this.advanceIntake = advanceIntake;
        this.reverseIntake = reverseIntake;
        this.advanceShooter = advanceShooter;
        maxCurrent = 0;
        peaked = false;
        touchedShooter = false;
        shooterCurrent = 0;

        addRequirements(intake, shooter); // might want to remove If I understand this correctly
    }

    public AltImprovedFeed(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        this.advanceIntake = new AdvanceIntake(intake, IntakeConstants.FEED_DISTANCE);
        this.reverseIntake = new AdvanceIntake(intake, IntakeConstants.REVERSE_DISTANCE);
        this.advanceShooter = new AdvanceShooter(shooter, ShooterConstants.ADVANCE_DISTANCE);
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
        if ((current < maxCurrent || (Timer.getFPGATimestamp() - startTime) > IntakeConstants.FEED_TIME) && !peaked) {
            peaked = true;
            reverseIntake.schedule();
        }

        if ((reverseIntake.isFinished() || (!reverseIntake.isScheduled() && peaked)) && !touchedShooter) {
            if (!advanceIntake.isScheduled()) {
                advanceIntake.schedule();
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
        return touchedShooter && (advanceShooter.isFinished() || advanceIntake.isFinished());
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
        shooter.stop();
    }
}

package frc.robot.commands.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TunableNumber;

public class ReversePIDFeed extends Command {
    private final Intake intake;
    private final double setPointDistance = IntakeConstants.REVERSE_DISTANCE;

    private double startTime;

    private final TunableNumber kP = new TunableNumber("PIDSetAngle kP", 0);
    private final TunableNumber kI = new TunableNumber("PIDSetAngle kI", 0);
    private final TunableNumber kD = new TunableNumber("PIDSetAngle kD", 0);

    private final PIDController distanceController = new PIDController(0, 0, 0);

    public ReversePIDFeed(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setVoltage(-IntakeConstants.FEED_VOLTAGE);
        startTime = Timer.getFPGATimestamp();
        //shooter.setVoltages(ShooterConstants.FEED_VOLTAGE, ShooterConstants.FEED_VOLTAGE);
    }

    @Override
    public void execute() {
        distanceController.setPID(kP.get(), kI.get(), kD.get());
        double pidDistance = distanceController.calculate(intake.getPosition(), setPointDistance);
        pidDistance = MathUtil.clamp(pidDistance, -0.5, 0.5); // need to configure bounds properly
        intake.setVoltage(pidDistance);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(intake.getPosition() - setPointDistance) < IntakeConstants.DISTANCE_TOLERANCE) || (Timer.getFPGATimestamp() - startTime) >= IntakeConstants.REVERSE_FEED_TIME;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }

}

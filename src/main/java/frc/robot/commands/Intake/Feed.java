package frc.robot.commands.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.TunableNumber;

public class Feed extends Command {
    private final Intake intake;

    private final TunableNumber kP = new TunableNumber("PIDSetAngle kP", 0);
    private final TunableNumber kI = new TunableNumber("PIDSetAngle kI", 0);
    private final TunableNumber kD = new TunableNumber("PIDSetAngle kD", 0);

    private final PIDController distanceController = new PIDController(0, 0, 0);

    // SET_POINT_DISTANCE is now a constant in IntakeConstants.java

    public Feed(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        distanceController.setPID(kP.get(), kI.get(), kD.get());
        double pidDistance = distanceController.calculate(intake.getPosition(), IntakeConstants.SET_POINT_DISTANCE);
        pidDistance = MathUtil.clamp(pidDistance, -0.5, 0.5); // need to configure bounds properly
        intake.setVoltage(pidDistance);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getPosition() - IntakeConstants.SET_POINT_DISTANCE) < IntakeConstants.DISTANCE_TOLERANCE;
    }

    @Override
    public void end(boolean disrupted) {
        intake.setVoltage(0);
    }

}

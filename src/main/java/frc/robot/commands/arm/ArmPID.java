package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

/**
 * Direct the ARM to rotate to a specific angle.
 * 
 * *NOTE:*
 * Does not have a built-in finish condition! Must be terminated manually!
 */
public class ArmPID extends Command {
    private final Arm arm;

    // TODO caluclate the proper values for each of these!!
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));
    private final ArmFeedforward feedforward = new ArmFeedforward(1, 1, 1, 1);
    private final PIDController controller = new PIDController(1, 1, 1);

    private State targetState;

    /**
     * Direct the ARM to rotate to a specific angle.
     * 
     * Handled by PID, feedforwad, and ✨ trapezoid motion profiling ✨.
     * 
     * @param arm the arm subsystem
     * @param targetAngle the target angle for the arm. In radians.
     */
    public ArmPID(Arm arm, double targetAngle) {
        this.arm = arm;
        this.targetState = new State(targetAngle, 0);

        addRequirements(arm);
    }

    @Override
    public void execute() {
        /**
         * Feed the current position state and the target state into the Trapezoid Motion
         * Profiling to smooth out the target setpoint speed.
         * The filtered speed (setpoint) is then fed through a feedforward to calculate
         * the voltage with a PIDController to smooth out any suspicious deviations 😈.
         */
        double currentPosition = arm.getArmPosition();
        State setpoint = profile.calculate(0.02, new State(currentPosition, arm.getArmVelocity()), targetState);
        double feedbackEffort = controller.calculate(currentPosition, setpoint.position);
        double feedforwardEffort = feedforward.calculate(setpoint.position, setpoint.velocity);

        arm.setVoltage(feedforwardEffort + feedbackEffort);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltage(0);
    }
}

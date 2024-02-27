package frc.robot.commands.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.TunableNumber;

/**
 * Direct the ARM to rotate to a specific angle.
 * 
 * *NOTE:*
 * Does not have a built-in finish condition! Must be terminated manually!
 */
public class ArmPID extends Command {
    private final Arm arm;

    // TODO caluclate the proper values for each of these!!
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 8);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.25, 3.8, 0.01);
    private final PIDController controller = new PIDController(7, 10, 0);

    private State targetState;
    private State initialState;
    private double startTime;

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
        controller.setIntegratorRange(-6, 6);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        initialState = new State(arm.getArmPosition(), arm.getArmVelocity());
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Update the target angle so that this command is reusable.
     * 
     * @param targetAngle
     */
    public void setTarget(double targetAngle) {
        targetState = new State(targetAngle, 0);
        initialize();
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

        Logger.recordOutput("Arm PID Target Position", targetState.position);

        //State setpoint = targetState;
        //State setpoint = profile.calculate(0.02, new State(currentPosition, arm.getArmVelocity()), targetState);
        State setpoint = profile.calculate(Timer.getFPGATimestamp() - startTime, initialState, targetState);

        Logger.recordOutput("Arm PID Setpoint Position", setpoint.position);
        Logger.recordOutput("Arm PID Setpoint Velocity", setpoint.velocity);

        double feedbackEffort = controller.calculate(currentPosition, setpoint.position);
        double feedforwardEffort = feedforward.calculate(setpoint.position, setpoint.velocity);

        Logger.recordOutput("Arm PID Feedback", feedbackEffort);
        Logger.recordOutput("Arm PID Feedforward", feedforwardEffort);


        arm.setVoltage(feedforwardEffort + feedbackEffort);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltage(0);
    }
}

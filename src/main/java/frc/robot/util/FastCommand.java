package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Run a command at a different frequency than usual
 */
public class FastCommand extends Command {
    private final Command command;
    private final Notifier notifier;
    private final double period;

    private final Object mutex = new Object();

    /**
     * @param command the command to run
     * @param period the period (in seconds) to run the command at
     */
    public FastCommand(Command command, double period) {
        this.command = command;
        this.period = period;
        notifier = new Notifier(() -> {
            synchronized (mutex) {
                command.execute();
            }
        });
    }

    @Override
    public void initialize() {
        command.initialize();
        notifier.startPeriodic(period);
    }

    @Override
    public boolean isFinished() {
        synchronized (mutex) {
            return command.isFinished();
        }
    }
}

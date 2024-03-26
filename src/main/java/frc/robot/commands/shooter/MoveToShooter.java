package frc.robot.commands.shooter;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Commands;
import frc.robot.commands.Intake.CurrentFeed;

public class MoveToShooter extends Command{
    private final AdvanceShooter advanceShooter;

    private final CurrentFeed currentFeed;

    public MoveToShooter(AdvanceShooter advanceShooter, CurrentFeed currentFeed) {
        this.advanceShooter = advanceShooter;
        this.currentFeed = currentFeed;
    }

    @Override
    public void initialize(){
        BooleanSupplier sup = () -> currentFeed.isFinished();
        Commands.backgroundTask(currentFeed, advanceShooter, sup);
    }

    @Override
    public boolean isFinished(){
        return advanceShooter.isFinished();
    }
}

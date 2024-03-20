package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.AdvanceShooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterPositionFeed extends ParallelRaceGroup {
    public ShooterPositionFeed(Intake intake, Shooter shooter) {
        addCommands(
            //new RaceFeed(intake),
            new WaitCommand(1).andThen(new AdvanceShooter(shooter, Units.inchesToMeters(14)))
        );
    }
}

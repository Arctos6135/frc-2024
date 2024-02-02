package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.shooter.Launch;

public class Score {
    public static Command scoreSpeaker(Arm arm, Shooter shooter, Intake intake) {
        // Issue #32 references a "stow angle" which was not found, SPEAKER_SCORING_POSITION created as a fix
        return new ArmPID(arm, ArmConstants.SPEAKER_SCORING_POSITION).andThen(new Launch(shooter, ShooterConstants.AMP_RPS)).andThen(new Feed(intake));
    }
}

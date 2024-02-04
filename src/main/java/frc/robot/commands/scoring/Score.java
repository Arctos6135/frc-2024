package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Commands;
import frc.robot.commands.Intake.CurrentFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.shooter.Launch;


public class Score {
    public static Command scoreSpeaker(Arm arm, Shooter shooter, Intake intake) {        
        return Commands.backgroundTask(
            new ArmPID(arm, ArmConstants.SPEAKER_SCORING_POSITION), 
            Commands.backgroundTask(new Launch(shooter, ShooterConstants.SPEAKER_RPS), new CurrentFeed(intake), () -> shooter.getVelocity() >= ShooterConstants.SPEAKER_RPS), 
            () -> arm.getArmPosition() >= ArmConstants.SPEAKER_SCORING_POSITION
        );
    }
}

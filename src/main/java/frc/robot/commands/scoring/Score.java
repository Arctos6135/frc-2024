package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.Intake.RaceFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.shooter.Launch;


public class Score {
    public static Command scoreSpeaker(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {        
        return new InstantCommand(() -> armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION))
            .andThen(new WaitUntilCommand(armPID::atTarget))
            .andThen(new Launch(shooter, ShooterConstants.SPEAKER_RPS)
            .raceWith(
                new WaitCommand(1.2).andThen(new Feed(intake).withTimeout(2))
            ));
    }

    public static Command scoreAmp(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {
        return new RaceFeed(shooter, intake)
            .withTimeout(1.3)
            .andThen(new InstantCommand(() -> armPID.setTarget(ArmConstants.AMP_SCORING_POSITION)))
            .andThen(new WaitUntilCommand(armPID::atTarget))
            .andThen(new Launch(shooter, ShooterConstants.AMP_RPS).withTimeout(0.5))
            .andThen(new InstantCommand(() -> armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION)));
    }

    public static void stop(Arm arm, Shooter shooter, Intake intake) {
        arm.setVoltage(0);
        shooter.stop();
        intake.setVoltage(0);
    }
}

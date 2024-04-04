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
// import frc.robot.commands.Intake.RaceFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.shooter.Launch;
import frc.robot.commands.shooter.ShooterPID;


// to improve auto
// timing
// - rev up shooter while driving
// - shoot while starting to drive to the next note
// accuracy
// - move robot on the field-side of the final note to avoid guardrail-robot collisions
// - run shooter backwards while intaking to avoid intaking too far
// - run shooter slightly faster
public class Score {
    public static Command scoreSpeaker(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {
        ShooterPID shooterPID = new ShooterPID(shooter, ShooterConstants.SPEAKER_RPS);
        
        return new InstantCommand(() -> armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION))
            .andThen(new WaitUntilCommand(armPID::atTarget))
            .andThen(shooterPID)
            .raceWith(
                shooterPID.waitUntilAtTarget().andThen(new Feed(intake).withTimeout(0.75))
            );
    }

    public static Command scoreAmp(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {
        return new InstantCommand(() -> armPID.setTarget(ArmConstants.AMP_SCORING_POSITION))
            .andThen(new WaitUntilCommand(armPID::atTarget))
            .andThen(new ShooterPID(shooter, ShooterConstants.AMP_RPS).withTimeout(0.5))
            .andThen(new InstantCommand(() -> armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION)));
    }

    public static Command ferryNote(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {
        return new InstantCommand(() -> armPID.setTarget(ArmConstants.FERRY_POSITION))
        .andThen(new WaitUntilCommand(armPID::atTarget))
        .andThen(new ShooterPID(shooter, ShooterConstants.FERRY_RPS).withTimeout(1))
        .andThen(new InstantCommand(()-> armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION)));
    }

    public static void stop(Arm arm, Shooter shooter, Intake intake) {
        arm.setVoltage(0);
        shooter.stop();
        intake.setVoltage(0);
    }
}

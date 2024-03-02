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
import frc.robot.util.Commands;
import frc.robot.commands.Intake.CurrentFeed;
import frc.robot.commands.Intake.RaceFeed;
import frc.robot.commands.Intake.ShooterPositionFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.shooter.Launch;


public class Score {
    public static Command scoreSpeaker(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {        
        return new InstantCommand(() -> armPID.setTarget(ArmConstants.AMP_SCORING_POSITION))
            .andThen(new WaitUntilCommand(armPID::atTarget))
            .andThen(new Launch(shooter, ShooterConstants.AMP_RPS))
            .andThen(new WaitCommand(5))
            .andThen(new RaceFeed(intake).withTimeout(1));
    }

    public static Command scoreAmp(Arm arm, ArmPID armPID, Shooter shooter, Intake intake) {
        return new ShooterPositionFeed(intake, shooter)
            .andThen(new InstantCommand(() -> armPID.setTarget(ArmConstants.AMP_SCORING_POSITION)))
            .andThen(new WaitUntilCommand(armPID::atTarget))
            .andThen(new Launch(shooter, ShooterConstants.AMP_RPS));
    }

    public static void stop(Arm arm, Shooter shooter, Intake intake) {
        arm.setVoltage(0);
        shooter.stop();
        intake.setVoltage(0);
    }
}

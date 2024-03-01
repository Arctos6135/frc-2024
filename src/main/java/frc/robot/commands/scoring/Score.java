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
            Commands.backgroundTask(new Launch(shooter, ShooterConstants.SPEAKER_RPS), new CurrentFeed(intake, shooter), () -> shooter.getVelocity() >= ShooterConstants.SPEAKER_RPS), //TODO: replace current feed with whatever feed we end up going with.
            () -> arm.getArmPosition() >= ArmConstants.SPEAKER_SCORING_POSITION
        );
    }

    public static Command scoreAmp(Arm arm, Shooter shooter, Intake intake) {
        return Commands.backgroundTask(
            new ArmPID(arm, ArmConstants.AMP_SCORING_POSITION),
            Commands.backgroundTask(new Launch(shooter, ShooterConstants.AMP_RPS), new CurrentFeed(intake, shooter), () -> shooter.getVelocity() >= ShooterConstants.AMP_RPS), //TODO: replace current feed with whatever feed we end up going with.
            () -> arm.getArmPosition() >= ArmConstants.AMP_SCORING_POSITION
        );
    }

    public static void stop(Arm arm, Shooter shooter, Intake intake) {
        arm.setVoltage(0);
        shooter.stop();
        intake.setVoltage(0);
    }
}

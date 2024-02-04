package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.Intake.CurrentFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.shooter.Launch;


public class Score {
    public static Command scoreSpeaker(Arm arm, Shooter shooter, Intake intake) {        
        return new ArmPID(arm, ArmConstants.SPEAKER_SCORING_POSITION)
            .raceWith(new WaitUntilCommand(() -> 
                checkPosition(arm, ArmConstants.SPEAKER_SCORING_POSITION))
                    .andThen(new CurrentFeed(intake))
                    .andThen(new Launch(shooter, ShooterConstants.SPEAKER_RPS)))
        ;
    }

    public static boolean checkPosition(Arm arm, double targetPosition) {
        // adjust error value
        double error = 0.1;

        // check if the arm position is in the range:
        // [targetPosition - error, targetPosition + error]
        if (arm.getArmPosition() >= targetPosition - error) {
            if (arm.getArmPosition() <= targetPosition + error) {
                return true;
            }
        }
        return false;
    }
}

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.winch.Winch;

public class Climb extends Command {
    public static Command Climb(Arm arm, Winch winch) {
        return new InstantCommand(() -> arm.setVoltage(-12))
            .andThen(new InstantCommand(() -> arm.setIdleMode(IdleMode.kBrake)))
            .andThen(new InstantCommand(() -> winch.setVoltage(12)))
            .andThen(new InstantCommand(() -> winch.setIdleMode(IdleMode.kBrake)));
    }
}

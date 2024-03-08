package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.winch.Winch;

public class Climb extends Command {
    private final Arm arm;
    private final Winch winch;
    private final XboxController toRumble;

    public Climb(Arm arm, Winch winch, XboxController toRumble) {
        this.arm = arm;
        this.winch = winch;
        this.toRumble = toRumble;

        addRequirements(arm, winch);
    }

    @Override
    public void initialize() {
        arm.setIdleMode(IdleMode.kBrake);
        winch.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void execute() {
        toRumble.setRumble(RumbleType.kBothRumble, 1);
        arm.setVoltage(-12);
        winch.setVoltage(12);
    }
    
    @Override
    public void end(boolean i) {
        arm.setVoltage(0);
        winch.setVoltage(0);
    }
}

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.winch.Winch;

public class Climb extends Command {
    private final Arm arm;
    private final Winch winch;
    public Climb(Arm arm, Winch winch) {
        this.arm = arm;
        this.winch = winch;

        addRequirements(arm, winch);
    }

    @Override
    public void initialize() {
        arm.setIdleMode(IdleMode.kBrake);
        winch.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void execute() {
        arm.setVoltage(-12);
        winch.setVoltage(12);
    }
    
    @Override
    public void end(boolean i) {
        // EXPLICITLY DOES NOTHING: EVERYTHING WILL KEEP RUNNING
    }
}

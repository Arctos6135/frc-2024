package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.WinchConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.winch.Winch;

public class Climb extends Command {
    private final Arm arm;
    private final Winch winch;
    private final XboxController toRumble;

    private double startWinchAngle;

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

        startWinchAngle = winch.getPosition();
    }

    @Override
    public void execute() {
        toRumble.setRumble(RumbleType.kBothRumble, 1);
        arm.setVoltage(-12);

        if (Math.abs(startWinchAngle - winch.getPosition()) <= WinchConstants.CORD_LENGTH) {
            winch.setVoltage(8);
        } else {
            winch.setVoltage(0);
            System.out.printf("Stopped at %s, started at %s\n", winch.getPosition(), startWinchAngle);
        }
    }
    
    @Override
    public void end(boolean i) {
        arm.setVoltage(0);
        winch.setVoltage(0);

        toRumble.setRumble(RumbleType.kBothRumble, 0);
    }
}

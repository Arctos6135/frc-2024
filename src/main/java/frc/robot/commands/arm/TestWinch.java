package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.WinchConstants;
import frc.robot.subsystems.winch.Winch;

public class TestWinch extends Command {
    private final Winch winch;
    private double startingPosition;

    public TestWinch(Winch winch) {
        this.winch = winch;
    }

    @Override
    public void initialize() {
        startingPosition = winch.getPosition();
    }

    @Override
    public void execute() {
        winch.setVoltage(-1);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(startingPosition - winch.getPosition()) >= 1.5;
    }

    @Override
    public void end(boolean i) {
        
    }
}

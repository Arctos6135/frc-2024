package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class DrivingIntake extends Command {
    private final Intake intake;
    private final XboxController controller;

    public DrivingIntake(Intake intake, XboxController controller) {
        this.intake = intake;
        this.controller = controller;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setVoltage((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())*12);
    }

    @Override
    public void end(boolean i) {
        intake.setVoltage(0);
    }
}
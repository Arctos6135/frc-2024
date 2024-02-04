package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Commands {
    public static Command backgroundTask(Command background, Command task, BooleanSupplier startingCondition) {
        return background.raceWith(new WaitUntilCommand(startingCondition).andThen(task));
    }
}

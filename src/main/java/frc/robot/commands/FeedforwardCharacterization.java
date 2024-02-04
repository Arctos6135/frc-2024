package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.commands.FeedForwardCharacterization.State;

// Slowly ramps up to full speed forward, decelerates down to 0, then drives backwards at full speed.
// The drivetrain should never go more than `config.maxDistance` meters ahead or behind where it started.
public class FeedforwardCharacterization extends Command {
    
    private class Data {
        private final ArrayList<Double> voltages = new ArrayList<>();
        private final ArrayList<Double> velocities = new ArrayList<>();
        private final ArrayList<Double> accelerations = new ArrayList<>();

        public void accept(double voltage, double velocity, double acceleration) {
            voltages.add(voltage);
            velocities.add(velocity);
            accelerations.add(acceleration);
        }

        public void logCSV(String name) {
            try {
                File outputFile;
                if (LoggedRobot.isReal()) {
                    outputFile = new File("/media/sda1/" + name + "Feedforward.csv");
                } else if (LoggedRobot.isSimulation()) {
                    outputFile = new File("./" + name + "Feedforward.csv");
                } else {
                    outputFile = new File("./" + name + "Feedforward.csv.replay");
                }
                outputFile.createNewFile();
                BufferedWriter out = new BufferedWriter(new FileWriter(outputFile));
                out.write("voltage,velocity,acceleration");
                out.newLine();
                for (int i = 0; i < voltages.size(); i++) {
                    out.write(String.format("%f,%f,%f", voltages.get(i), velocities.get(i), accelerations.get(i)));
                    out.newLine();
                }
                out.close();
            } catch (Exception e) {
                DriverStation.reportError("Failed to create file to write feedforward data", e.getStackTrace());
            }

        }
    }

    public record Config(
        DoubleConsumer voltage, 
        DoubleSupplier position, 
        DoubleSupplier velocity,
        // volts per second
        double rampRate,
        double maxVoltage,
        double holdTime,
        double maxDistance,
        String name
    ) {}

    private final Config config;
    private final Data data = new Data();
    private boolean rampingUp = true;
    // seconds
    private double startTime;
    private double currentVoltage = 0;
    private double previousVelocity = 0;
    private double startDistance;

    public FeedforwardCharacterization(Config config, Subsystem... requirements) {
        this.config = config;

        for (Subsystem subsystem : requirements) {
            addRequirements(subsystem);
        }
    }

    @Override
    public void initialize() {
        startDistance = config.position.getAsDouble();
    }

    @Override
    public void execute() {
        double velocity = config.velocity.getAsDouble();
        double acceleration = (velocity - previousVelocity) / 0.02;
        previousVelocity = velocity;
        data.accept(currentVoltage, velocity, acceleration);
        Logger.recordOutput("Feedforward Current Voltage", currentVoltage);
        Logger.recordOutput("Feedforward Velocity", velocity);
        Logger.recordOutput("Feedforward Acceleration", acceleration);

        if (rampingUp) {
            currentVoltage += config.rampRate * 0.02;
            if (currentVoltage >= config.maxVoltage) {
                rampingUp = false;
            }
        } else {
            currentVoltage = -config.maxVoltage;
        }
        config.voltage.accept(currentVoltage);
    }

    @Override
    public boolean isFinished() {
        double elapsed = Timer.getFPGATimestamp() - startTime;
        double distance = Math.abs(startDistance - config.position.getAsDouble());
        boolean doneHolding = (elapsed >= config.holdTime) && (!rampingUp);
        boolean tooFar = distance >= config.maxDistance;
        boolean tooFarBack = config.position.getAsDouble() <= startDistance;

        return doneHolding || tooFar || tooFarBack;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("ending with position %s\n", config.position.getAsDouble());

        config.voltage.accept(0);
        data.logCSV(config.name);
    }
}
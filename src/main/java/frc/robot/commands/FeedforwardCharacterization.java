package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

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
                File outputFile = new File("/media/sda1/" + name + "Feedforward.csv");
                outputFile.createNewFile();
                BufferedWriter out = new BufferedWriter(new FileWriter(outputFile));
                out.write("voltage,velocity,acceleration");
                for (int i = 0; i < voltages.size(); i++) {
                    out.write(String.format("%f,%f,%f", voltages.get(i), velocities.get(i), accelerations.get(i)));
                    out.newLine();
                }
                out.close();
            } catch (Exception e) {
                DriverStation.reportError("Failed to create file to write feedforward data", null);
                e.printStackTrace();
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

    private enum State {
        RampingUp,
        RampingDown,
        HoldingNegative
    }

    private final Config config;
    private final Data data = new Data();
    private State state = State.RampingUp;
    // seconds
    private double startTime;
    private double currentVoltage = 0;
    private double prevVelocity = 0;
    private double startDistance;

    public FeedforwardCharacterization(Config config) {
        this.config = config;
    }

    @Override
    public void initialize() {
        startDistance = config.position.getAsDouble();
    }

    @Override
    public void execute() {
        double velocity = config.velocity.getAsDouble();
        double acceleration = (velocity - prevVelocity) / 0.02;
        prevVelocity = velocity;
        data.accept(currentVoltage, velocity, acceleration);

        if (state == State.RampingUp) {
            currentVoltage += config.rampRate * 0.02;
            if (currentVoltage >= config.maxVoltage) {
                state = State.RampingDown;
            }
        } else if (state == State.RampingDown) {
            currentVoltage -= config.rampRate * 0.02;
            if (currentVoltage <= 0) {
                startTime = Timer.getFPGATimestamp();
                state = State.HoldingNegative;
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
        boolean doneHolding = elapsed >= config.holdTime;
        boolean tooFar = distance >= config.maxDistance;
        
        return doneHolding || tooFar;
    }

    @Override
    public void end(boolean interrupted) {
        config.voltage.accept(0);
        data.logCSV(config.name);
    }
}

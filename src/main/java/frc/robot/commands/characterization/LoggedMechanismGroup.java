package frc.robot.commands.characterization;

import java.util.ArrayList;
import java.util.List;

public class LoggedMechanismGroup {
    private final List<LoggedMechanism> mechanisms;
    private double leastDistance = 0;
    private double greatestDistance = 0;

    public LoggedMechanismGroup(List<LoggedMechanism> mechanisms) {
        this.mechanisms = mechanisms;
    }

    public LoggedMechanismGroup(LoggedMechanism ...mechanisms) {
        ArrayList<LoggedMechanism> mechanismList = new ArrayList<>();

        for (LoggedMechanism loggedMechanism : mechanisms) {
            mechanismList.add(loggedMechanism);
        }

        this.mechanisms = mechanismList;
    }

    public void initialize() {
        for (LoggedMechanism loggedMechanism : mechanisms) {
            loggedMechanism.initialize();
        }
    }

    public void setVoltage(double voltage) {
        leastDistance = Double.MAX_VALUE;
        greatestDistance = 0;
        for (LoggedMechanism loggedMechanism : mechanisms) {
            double distance = loggedMechanism.update(voltage);
            leastDistance = Math.min(leastDistance, distance);
            greatestDistance = Math.max(greatestDistance, distance);
        }
    }

    public double getLeastDistance() {
        return leastDistance;
    }

    public double getGreatestDistance() {
        return greatestDistance;
    }
}

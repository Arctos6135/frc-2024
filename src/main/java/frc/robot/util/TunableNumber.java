package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private static final String TABLE_ROOT = "TunableNumbers";

    private final DoubleEntry entry;
    private double lastEntry;

    private final String name;
    private final double defaultValue;

    public TunableNumber(String name, double defaultValue) {
        entry = NetworkTableInstance.getDefault().getTable(TABLE_ROOT).getDoubleTopic(name).getEntry(defaultValue);
        lastEntry = defaultValue;

        this.name = name;
        this.defaultValue = defaultValue;

        SmartDashboard.putNumber(name, defaultValue);
    }

    public double get() {
        return SmartDashboard.getNumber(name, defaultValue);
    }

    public static void ifChanged(Runnable onChange, TunableNumber... watching) {
        boolean changed = false;
        for (TunableNumber tunableNumber : watching) {
            if (tunableNumber.get() != tunableNumber.lastEntry) {
                tunableNumber.lastEntry = tunableNumber.get();
                changed = true;
            }
        }
        if (changed) {
            onChange.run();
        }
    }
}

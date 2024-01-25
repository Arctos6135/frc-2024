package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableNumber {
    private static final String TABLE_ROOT = "TunableNumbers";

    private final DoubleEntry entry;

    public TunableNumber(String name, double defaultValue) {
        entry = NetworkTableInstance.getDefault().getTable(TABLE_ROOT).getDoubleTopic(name).getEntry(defaultValue);
    }

    public double get() {
        return entry.get();
    }
}

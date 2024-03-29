package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class SparkMaxCANChecker {
    private final CANSparkMax sparkMax;
    private final int ID;

    public SparkMaxCANChecker(CANSparkMax sparkMax) {
        this.sparkMax = sparkMax;
        ID = sparkMax.getDeviceId();
    }

    public void update() {
        Logger.recordOutput("CAN/" + ID, sparkMax.getLastError());
    }
}

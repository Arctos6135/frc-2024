package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.CANBus;

public class IntakeIOSparkMax {
    private final CANSparkMax topMaster = new CANSparkMax(CANBus.INTAKE_TOP_MASTER, MotorType.kBrushless);
    private final CANSparkMax bottomMaster = new CANSparkMax(CANBus.INTAKE_BOTTOM_MASTER, MotorType.kBrushless);
    private final CANSparkMax topFollower = new CANSparkMax(CANBus.INTAKE_TOP_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax bottomFollower = new CANSparkMax(CANBus.INTAKE_BOTTOM_FOLLOWER, MotorType.kBrushless);

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    public IntakeIOSparkMax() {
        this.topFollower.follow(this.topMaster);
        this.bottomFollower.follow(this.bottomMaster);

        this.topMaster.setInverted(true);

        this.topMaster.setIdleMode(IdleMode.kBrake);
        this.bottomMaster.setIdleMode(IdleMode.kBrake);
    }
}

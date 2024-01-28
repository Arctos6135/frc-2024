package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;


public class Arm extends SubsystemBase {
    private final ArmIO io;

    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    private final Mechanism2d mechanism = new Mechanism2d(42, 42);
    private final MechanismRoot2d root = mechanism.getRoot("Arm", 21, 21);
    private final MechanismLigament2d shortArm = new MechanismLigament2d("Short Arm", 8, 60, 10, new Color8Bit(255, 0, 0));
    private final MechanismLigament2d longArm = new MechanismLigament2d("Long Arm", 15, -60);

    public Arm(ArmIO io) {
        this.io = io;
        root.append(shortArm);
        shortArm.append(longArm);
    }

    @Override
    public void periodic() {
        // This tells our arm (either real or simulated) to update our class with all the sensor data.
        io.updateInputs(inputs);

        // Update mechanism
        shortArm.setAngle(inputs.position + 60);
        longArm.setAngle(-60);

        // Log all the sensor data.
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm Mechanism", mechanism);

    }

    public double getArmPosition() {
        return inputs.position;
    }

    public double getArmVelocity() {
        return inputs.velocity;
    }

    public void setVoltage(double voltage) {
        Logger.recordOutput("Arm Voltage", voltage);
        io.setVoltage(voltage);
    }
}
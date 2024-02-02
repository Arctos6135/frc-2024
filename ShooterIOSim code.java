package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.ShooterConstants;


public class FlywheelSim extends ShooterIO {
    private final ShooterIO io;
    public static final jKgMetersSquared;
    public static final shooter.update(0.02);
    private final Flywheel shooter = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.GEARBOX_RATIO, MOMENT_OF_INERTIA);

    FlywheelSim(DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N1,N1> measurementStdDevs){
        public void updateInputs(FlywheelSim inputs){
            shooter.update(0.02);
            io.updateInputs(inputs);
            Logger.processInputs("Shooter", inputs);
        }
    }


    @Override
    public void setVoltages(double left, double right) {
        drive.setInputs(left, right);
    }


    /* To set voltages and call the method for the results. */
    private void setInputVoltage(double leftVoltage, double rightVoltage) {
    Logger.recordOutput("Shooter Left Voltage:", leftVoltage);
    Logger.recordOutput("Shooter Right Voltage:", rightVoltage);
    System.out.println("Voltages have been set!", io.setVoltages(leftVoltage, rightVoltage));
    }



}
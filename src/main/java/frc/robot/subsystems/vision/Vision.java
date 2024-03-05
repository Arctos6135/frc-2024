package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    
    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();
    
    @Override
    public void periodic() {
    }

    public Vision() {
	this.io = new VisionIO();
    }
}
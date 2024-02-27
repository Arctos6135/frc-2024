package frc.robot.subsystems.vision;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
	public final boolean hasTarget() {
		return true;
	}

	// in meters
	public final double getNoteDistance() {
		return 0.0;
	}

	// in radians
	public final double getNoteAngle() {
		return 0.0;
	}
}
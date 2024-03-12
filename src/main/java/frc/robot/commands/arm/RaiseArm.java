package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;

public class RaiseArm extends Command{

    private final Arm arm;
    private final XboxController operatorController;
    private double targetAngle;
    private ArmPID armPID;

    public RaiseArm(Arm arm, XboxController operatorController, double targetAngle, ArmPID armPID){
        this.arm = arm;
        this.operatorController = operatorController;
        this.targetAngle = targetAngle;
        this.armPID = new ArmPID(arm, targetAngle);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(this.armPID.atTarget()){
            this.targetAngle += operatorController.getRawAxis(XboxController.Axis.kLeftY.value) * ArmConstants.CLIMB_SENSITIVITY;
            armPID.setTarget(targetAngle);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
        armPID.end(true);
    }
}

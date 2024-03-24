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

    public RaiseArm(Arm arm, XboxController operatorController, ArmPID armPID){
        this.arm = arm;
        this.operatorController = operatorController;
        this.armPID = armPID;
        this.targetAngle = ArmConstants.STARTING_POSITION + 1.35;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (this.armPID.atTarget()){
            this.targetAngle -= operatorController.getLeftY() * ArmConstants.CLIMB_SENSITIVITY;
            armPID.setTarget(targetAngle);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted){}
}

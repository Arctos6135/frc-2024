// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.driving.PIDSetAngle;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PositionConstants;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.shooter.*;

public class RobotContainer {
    // Xbox controllers
    private final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
    private final XboxController operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER);

    // Subsystems
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Arm arm;
    private final Shooter shooter;

    // Sendable choosers (for driveteam to select autos and positions)
    public LoggedDashboardChooser<PathPlannerAuto> autoChooser;
    public LoggedDashboardChooser<Pose2d> positionChooser;

    // Creates an option on the dashboard to turn manual intake on and off.
    public LoggedDashboardBoolean manualIntake;
    public LoggedDashboardBoolean disableArm;
    public LoggedDashboardBoolean resetArmEncoder;

    // Commands
    private final TeleopDrive teleopDrive;

    public RobotContainer() {
        // Creates a real robot.
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOSparkMax());
            intake = new Intake(new IntakeIOSparkMax());
            arm = new Arm(new ArmIO());
            shooter = new Shooter(new ShooterIO());
        }
        // Creates a simulated robot.
        else if (RobotBase.isSimulation()) {
            drivetrain = new Drivetrain(new DrivetrainIOSim());
            arm = new Arm(new ArmIOSim());
            
            // Will be changed to IntakeIOSim when it is programmed.
            intake = new Intake(new IntakeIO());
            // Will be changed to ShooterIOSim when it is programmed.
            shooter = new Shooter(new ShooterIO());
        } 
        // Creates a replay robot.
        else {
            drivetrain = new Drivetrain(new DrivetrainIO());
            intake = new Intake(new IntakeIO());
            arm = new Arm(new ArmIO());
            shooter = new Shooter(new ShooterIO());
        }

        teleopDrive = new TeleopDrive(drivetrain, driverController);
        drivetrain.setDefaultCommand(teleopDrive);

        autoChooser = new LoggedDashboardChooser<PathPlannerAuto>("auto chooser");
        positionChooser = new LoggedDashboardChooser<Pose2d>("position chooser");

        autoChooser.addDefaultOption("Test Auto", new PathPlannerAuto("Test Auto"));
        positionChooser.addDefaultOption("default pose", PositionConstants.POSE1);

        // Placeholders until autos are coded.
        autoChooser.addOption("Quarter Circle", new PathPlannerAuto("Quarter Circle"));
        autoChooser.addOption("1 Meter Forward", new PathPlannerAuto("1 Meter Forward"));

        // Placeholders until positions are configured.

        positionChooser.addOption("Position 1", PositionConstants.POSE2);
        positionChooser.addOption("Position 2", PositionConstants.POSE2);
        positionChooser.addOption("Position 3", PositionConstants.POSE3);

        manualIntake = new LoggedDashboardBoolean("manual intake");
        disableArm = new LoggedDashboardBoolean("disable arm");
        resetArmEncoder = new LoggedDashboardBoolean("reset arm encoder");

        configureBindings();
    }

    private void configureBindings() {
        // Binds precision drive toggling to driver's right bumper.
        new Trigger(() -> driverController.getRightBumper())
            .onTrue(new InstantCommand(() -> teleopDrive.setPrecisionDrive(true)))
            .onFalse(new InstantCommand(() -> teleopDrive.setPrecisionDrive(false)));

        // Binds macros for orienting robot turning to driver's dpad.
        new Trigger(() -> driverController.getPOV() == 0).onTrue(new PIDSetAngle(drivetrain, 0));
        new Trigger(() -> driverController.getPOV() == 45).onTrue(new PIDSetAngle(drivetrain, Math.PI / 4));
        new Trigger(() -> driverController.getPOV() == 90).onTrue(new PIDSetAngle(drivetrain, Math.PI / 2));
        new Trigger(() -> driverController.getPOV() == 135).onTrue(new PIDSetAngle(drivetrain, (3 * Math.PI) / 4));
        new Trigger(() -> driverController.getPOV() == 180).onTrue(new PIDSetAngle(drivetrain, Math.PI));
        new Trigger(() -> driverController.getPOV() == 225).onTrue(new PIDSetAngle(drivetrain, (5 * Math.PI) / 4));
        new Trigger(() -> driverController.getPOV() == 270).onTrue(new PIDSetAngle(drivetrain, (3 * Math.PI) / 2));
        new Trigger(() -> driverController.getPOV() == 315).onTrue(new PIDSetAngle(drivetrain, (7 * Math.PI) / 4));

        // Changed the intake triggers to driver controller for testing.
        // TODO change back to the operator controller.
        // Sets the right bumper to turn the intake on until released.
        new Trigger(() -> driverController.getAButtonPressed())
        .onTrue(new InstantCommand(() -> intake.setVoltage(IntakeConstants.VOLTAGE)))
        .onFalse(new InstantCommand(() -> intake.setVoltage(0)));
        
        // Binds the left bumper to run intake in reverse until released.
        new Trigger(() -> driverController.getAButtonPressed())
        .onTrue(new InstantCommand(() -> intake.setVoltage(-IntakeConstants.VOLTAGE)))
        .onFalse(new InstantCommand(() -> intake.setVoltage(0)));

        // Binds auto intake to the a button.
        new Trigger(() -> operatorController.getAButtonPressed()).onTrue(new IntakePiece(intake));

        // Binds moving the arm to the operator's d-pad if the arm is enabled.
        if (!disableArm.get()) {
            new Trigger(() -> operatorController.getPOV() == 0)
            .onTrue(new ArmPID(arm, arm.getArmPosition() + Math.PI / 180))
            .onFalse(new ArmPID(arm, arm.getArmPosition()));

            new Trigger(() -> operatorController.getPOV() == 180)
            .onTrue(new ArmPID(arm, arm.getArmPosition() - Math.PI / 180))
            .onFalse(new ArmPID(arm, arm.getArmPosition()));
        }

        // TODO Configure shooter launch button :)
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

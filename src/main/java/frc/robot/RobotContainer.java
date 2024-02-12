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
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.driving.PIDSetAngle;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.IntakeConstants;
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

    // Creates an option on the dashboard to turn manual intake on and off.
    public LoggedDashboardBoolean manualIntake;

    // Commands
    private final TeleopDrive teleopDrive;

    public RobotContainer() {
        // Creates a real robot.
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOSparkMax());
            intake = new Intake(new IntakeIO());
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

        autoChooser.addDefaultOption("Test Auto", new PathPlannerAuto("Test Auto"));

        // Placeholders until autos are coded.
        autoChooser.addOption("Auto1", new PathPlannerAuto("Test Auto"));
        autoChooser.addOption("Auto2", new PathPlannerAuto("Test Auto"));
        autoChooser.addOption("Auto3", new PathPlannerAuto("Test Auto"));

        manualIntake = new LoggedDashboardBoolean("manual intake");

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

        if (manualIntake.get()) {
            // Sets the a button to turn the intake on until released.
            new Trigger(() -> operatorController.getAButtonPressed())
            .onTrue(new InstantCommand(() -> intake.setVoltage(IntakeConstants.VOLTAGE)))
            .onFalse(new InstantCommand(() -> intake.setVoltage(0)));
        }

        else {
            // Sets the a button to run the intake command.
            new Trigger(() -> operatorController.getAButtonPressed()).onTrue(new IntakePiece(intake));
        }

        // TODO Configure shooter launch button :)
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

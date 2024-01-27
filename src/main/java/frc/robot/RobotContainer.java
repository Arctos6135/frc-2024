// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.driving.PIDSetAngle;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.IntakePiece;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DrivetrainIO;
import frc.robot.subsystems.Drivetrain.Drivetrain.DrivetrainIOSim;
import frc.robot.subsystems.Drivetrain.Drivetrain.DrivetrainIOSparkMax;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;;

public class RobotContainer {
    // Xbox controllers
    private final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
    private final XboxController operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER);

    // Subsystems
    private final Drivetrain drivetrain;
    private final Intake intake;

    // Commands
    private final TeleopDrive teleopDrive;

    public RobotContainer() {
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOSparkMax());
            intake = new Intake(new IntakeIOSparkMax());
        } else if (RobotBase.isSimulation()) {
            drivetrain = new Drivetrain(new DrivetrainIOSim());
            // Will be changed to IntakeIOSim when it is programmed.
            intake = new Intake(new IntakeIO());
        } else {
            drivetrain = new Drivetrain(new DrivetrainIO());
            intake = new Intake(new IntakeIO());
        }

        teleopDrive = new TeleopDrive(drivetrain, driverController);
        drivetrain.setDefaultCommand(teleopDrive);

                configureBindings();

    }

    private void configureBindings() {
        new Trigger(() -> driverController.getRightBumper())
            .onTrue(new InstantCommand(() -> teleopDrive.setPrecisionDrive(true)))
            .onFalse(new InstantCommand(() -> teleopDrive.setPrecisionDrive(false)));

        new Trigger(() -> driverController.getPOV() == 0).onTrue(new PIDSetAngle(drivetrain, 0));
        new Trigger(() -> driverController.getPOV() == 45).onTrue(new PIDSetAngle(drivetrain, Math.PI / 4));
        new Trigger(() -> driverController.getPOV() == 90).onTrue(new PIDSetAngle(drivetrain, Math.PI / 2));
        new Trigger(() -> driverController.getPOV() == 135).onTrue(new PIDSetAngle(drivetrain, (3 * Math.PI) / 4));
        new Trigger(() -> driverController.getPOV() == 180).onTrue(new PIDSetAngle(drivetrain, Math.PI));
        new Trigger(() -> driverController.getPOV() == 225).onTrue(new PIDSetAngle(drivetrain, (5 * Math.PI) / 4));
        new Trigger(() -> driverController.getPOV() == 270).onTrue(new PIDSetAngle(drivetrain, (3 * Math.PI) / 2));
        new Trigger(() -> driverController.getPOV() == 315).onTrue(new PIDSetAngle(drivetrain, (7 * Math.PI) / 4));

        new Trigger(() -> operatorController.getAButtonPressed()).onTrue(new IntakePiece(intake));
    }

    public Command getAutonomousCommand() {
        return drivetrain.characterize();
    }
}

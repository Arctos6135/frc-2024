// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainIO;
import frc.robot.subsystems.DrivetrainIOSim;
import frc.robot.subsystems.DrivetrainIOSparkMax;

public class RobotContainer {
  private final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
  private final XboxController operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER);
  public final Drivetrain drivetrain;

  public RobotContainer() {
    configureBindings();

    if (RobotBase.isReal()) {
      this.drivetrain = new Drivetrain(new DrivetrainIOSparkMax());
    } else if (RobotBase.isSimulation()) {
      this.drivetrain = new Drivetrain(new DrivetrainIOSim());
    } else {
      this.drivetrain = new Drivetrain(new DrivetrainIO());
    }

    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverController));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

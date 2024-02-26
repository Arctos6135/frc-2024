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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.driving.PIDSetAngle;
import frc.robot.commands.arm.ArmPID;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.commands.scoring.Score;
import frc.robot.commands.shooter.Launch;
import frc.robot.constants.ArmConstants;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PositionConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.ControllerConstants;
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
    public LoggedDashboardChooser<Command> autoChooser;
    public LoggedDashboardChooser<Pose2d> positionChooser;

    // Creates an option on the dashboard to turn manual intake on and off.
    public LoggedDashboardBoolean manualIntake;
    public LoggedDashboardBoolean disableArm;
    public LoggedDashboardBoolean resetArmEncoder;

    // Commands
    private final TeleopDrive teleopDrive;
    private final ArmPID armPID;
    private final Score score;

    public RobotContainer() {
        // Creates a real robot.
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOSparkMax());
            intake = new Intake(new IntakeIOSparkMax());
            arm = new Arm(new ArmIOSparkMax());
            shooter = new Shooter(new ShooterIOSparkMax());
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

        score = new Score();
        teleopDrive = new TeleopDrive(drivetrain, driverController);
        drivetrain.setDefaultCommand(teleopDrive);

        armPID = new ArmPID(arm, ArmConstants.STARTING_POSITION);
        //arm.setDefaultCommand(armPID);

        autoChooser = new LoggedDashboardChooser<Command>("auto chooser");
        positionChooser = new LoggedDashboardChooser<Pose2d>("position chooser");

        autoChooser.addDefaultOption("Drivetrain Velocity", drivetrain.characterizeVelocity());
        positionChooser.addDefaultOption("Position 1", PositionConstants.POSE1);

        // Placeholders until autos are coded.
        autoChooser.addOption("Quarter Circle", new PathPlannerAuto("Quarter Circle"));
        autoChooser.addOption("1 Meter Forward", new PathPlannerAuto("1 Meter Forward"));

        // Characterization routines.
        autoChooser.addOption("Drivetrain Velocity", drivetrain.characterizeVelocity());
        autoChooser.addOption("Drivetrain Acceleration", drivetrain.characterizeAcceleration());

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
        // new Trigger(() -> driverController.getRightBumper())
        //     .onTrue(new InstantCommand(() -> teleopDrive.setPrecisionDrive(true)))
        //     .onFalse(new InstantCommand(() -> teleopDrive.setPrecisionDrive(false)));

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

        new Trigger(() -> driverController.getRightBumperPressed()).onTrue(new InstantCommand(() -> intake.setVoltage(IntakeConstants.VOLTAGE)));
        new Trigger(() -> driverController.getRightBumperReleased()).onTrue(new InstantCommand(() -> intake.setVoltage(0)));

        // // Binds the left bumper to run intake in reverse until released.
        new Trigger(() -> driverController.getLeftBumperPressed()).onTrue(new InstantCommand(() -> intake.setVoltage(-IntakeConstants.VOLTAGE)));
        new Trigger(() -> driverController.getLeftBumperReleased()).onTrue(new InstantCommand(() -> intake.setVoltage(0)));

        // Binds auto intake to the a button.
        //new Trigger(() -> operatorController.getAButtonPressed()).onTrue(new IntakePiece(intake));


        new Trigger(() -> driverController.getXButtonPressed()).whileTrue(new InstantCommand(() -> shooter.setVoltages(12, 12)));
        new Trigger(() -> driverController.getXButtonReleased()).whileTrue(new InstantCommand(() -> shooter.setVoltages(0, 0)));

        new Trigger(() -> driverController.getYButtonPressed()).whileTrue(new Launch(shooter, 1.5));
        new Trigger(() -> driverController.getYButtonReleased()).whileTrue(new Launch(shooter, 0));

        // The armPID is binded to the operator X and Y buttons. Check this.updateButtons() for more information.

        // TODO Configure shooter launch button :)

        // Binds the speaker shoot to the x button.
        // Temporarily bound to the driver controller.
        // new Trigger(() -> driverController.getXButtonPressed()).onTrue(score.scoreSpeaker(arm, shooter, intake));
        // new Trigger(() -> driverController.getXButtonReleased()).onTrue(new InstantCommand(() -> score.stop(arm, shooter, intake)));
        // new Trigger(() -> driverController.getYButtonPressed()).onTrue(score.scoreAmp(arm, shooter, intake));
        // new Trigger(() -> driverController.getYButtonReleased()).onTrue(new InstantCommand(() -> score.stop(arm, shooter, intake)));
    }

    /**
     * Runs on a periodic loop. Check Robot.java.
     */
    public void updateButtons() {
        if (operatorController.getXButton()) {
            arm.setVoltage(6); //armPID.setTarget(ArmConstants.AMP_SCORING_POSITION);
        } else if (operatorController.getYButton()) {
            armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION);
        } else {
            arm.setVoltage(0); //armPID.setTarget(ArmConstants.STARTING_POSITION);
        }
    }

    public Command getAutonomousCommand() {
        return new InstantCommand(() -> armPID.setTarget(ArmConstants.AMP_SCORING_POSITION)); //autoChooser.get();
    }
}

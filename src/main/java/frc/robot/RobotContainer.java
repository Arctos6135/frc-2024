// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Intake.CurrentFeed;
import frc.robot.commands.Intake.ImprovedFeeed;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.Intake.RaceFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.driving.PIDSetAngle;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.scoring.Score;
import frc.robot.commands.shooter.AdvanceShooter;
import frc.robot.commands.shooter.Launch;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PositionConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainIOSim;
import frc.robot.subsystems.drivetrain.DrivetrainIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;

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

    // Named Commands (for autos)
    public NamedCommands scoreSpeaker;
    public NamedCommands runIntake;

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
            intake = new Intake(new IntakeIOSim());
            // Will be changed to ShooterIOSim when it is programmed.
            shooter = new Shooter(new ShooterIOSim());
        } 
        // Creates a replay robot.
        else {
            drivetrain = new Drivetrain(new DrivetrainIO());
            intake = new Intake(new IntakeIO());
            arm = new Arm(new ArmIO());
            shooter = new Shooter(new ShooterIO());
        }

        // Named commands for autos.
        NamedCommands.registerCommand("scoreSpeaker", Score.scoreSpeaker(arm, shooter, intake));
        NamedCommands.registerCommand("stopScoring", new InstantCommand(() -> Score.stop(arm, shooter, intake)));
        NamedCommands.registerCommand("runIntake", new InstantCommand(() -> intake.setVoltage(12)));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.setVoltage(0)));

        teleopDrive = new TeleopDrive(drivetrain, driverController);
        drivetrain.setDefaultCommand(teleopDrive);

        armPID = new ArmPID(arm, ArmConstants.STARTING_POSITION);
        //arm.setDefaultCommand(armPID);

        autoChooser = new LoggedDashboardChooser<Command>("auto chooser");
        positionChooser = new LoggedDashboardChooser<Pose2d>("position chooser");

        autoChooser.addDefaultOption("Quarter Circle", new PathPlannerAuto("Quarter Circle"));
        positionChooser.addDefaultOption("Position 1", PositionConstants.POSE1);

        // Placeholders until autos are coded.
        autoChooser.addOption("Three Note Auto", new PathPlannerAuto("Three Note Auto"));
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
        Trigger driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        Trigger driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        Trigger driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
        Trigger driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
        Trigger driverX = new JoystickButton(driverController, XboxController.Button.kY.value);
        Trigger driverY = new JoystickButton(driverController, XboxController.Button.kY.value);

        // Binds precision drive toggling to driver's right bumper.
        // driverRightBumper
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

        Trigger operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
        Trigger operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
        Trigger operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
        Trigger operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
        Trigger operatorX = new JoystickButton(operatorController, XboxController.Button.kY.value);
        Trigger operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);

        // Binds auto intake to the a button.
        // operatorA.onTrue(new CurrentFeed(intake, shooter));
        operatorB.onTrue(new IntakePiece(intake));


        operatorX.whileTrue(new InstantCommand(() -> shooter.setVoltages(12, 12)));
        operatorX.whileFalse(new InstantCommand(() -> shooter.setVoltages(0, 0)));

        operatorY.whileTrue(new Launch(shooter, 1.5));
        operatorY.whileFalse(new Launch(shooter, 0));

        operatorLeftBumper.whileTrue(new RaceFeed(intake).raceWith(new WaitCommand(0.5).andThen(new AdvanceShooter(shooter, Units.inchesToMeters(ShooterConstants.ADVANCE_DISTANCE)))));
        operatorA.onTrue(new ImprovedFeeed(intake, shooter));

        // The armPID is binded to the operator X and Y buttons. Check this.updateButtons() for more information.

        // TODO Configure shooter launch button :)

        // Binds the speaker shoot to the x button.
        // Temporarily bound to the driver controller.
        // driverX.onTrue(score.scoreSpeaker(arm, shooter, intake));
        // driverX.onFalse(new InstantCommand(() -> score.stop(arm, shooter, intake)));
        // driverY.onTrue(score.scoreAmp(arm, shooter, intake));
        // driverY.onFalse(new InstantCommand(() -> score.stop(arm, shooter, intake)));
    }

    /**
     * Runs on a periodic loop. Check Robot.java.
     */
    public void updateButtons() {
        if (operatorController.getXButton()) {
            arm.setVoltage(4); //armPID.setTarget(ArmConstants.AMP_SCORING_POSITION);
            System.out.println("Pressing X");
        } else if (operatorController.getYButton()) {
            //armPID.setTarget(ArmConstants.SPEAKER_SCORING_POSITION);
            arm.setVoltage(-4);
        } else {
            arm.setVoltage(0); //armPID.setTarget(ArmConstants.STARTING_POSITION);
            System.out.println("Not pressing X");
        }

        // Runs the intake faster or slower depending on how the operator trigger is pulled.
        intake.setVoltage((operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()) * 12);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("1 Meter Forward");//new InstantCommand(() -> armPID.setTarget(Units.degreesToRadians(30)));//autoChooser.get();
    }
}

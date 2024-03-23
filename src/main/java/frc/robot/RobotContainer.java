// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import astrolabe.follow.AutoBuilder;
import astrolabe.follow.FollowPath;
import astrolabe.follow.AstrolabeLogger;
import astrolabe.follow.GlobalConfig;
import astrolabe.follow.FollowTrajectory;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.commands.Intake.ShooterPositionFeed;
// import frc.robot.commands.arm.ArmPID;
// import frc.robot.commands.driving.ProfiledPIDSetAngle;
//import frc.robot.commands.driving.ProfiledPIDSetAngle;
// import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.Intake.AltImprovedFeed;
import frc.robot.commands.Intake.DrivingIntake;
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.Intake.RaceFeed;
import frc.robot.commands.shooter.ReverseFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.arm.Climb;
// import frc.robot.commands.scoring.Score;
// import frc.robot.commands.arm.Climb;
import frc.robot.commands.arm.RaiseArm;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.scoring.Score;
import frc.robot.constants.*;
// import frc.robot.constants.ArmConstants;
// import frc.robot.constants.ControllerConstants;
// // import frc.robot.constants.PositionConstants;
// import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.ArmIO;
// import frc.robot.subsystems.arm.ArmIOSim;
// import frc.robot.subsystems.arm.ArmIOSparkMax;
// import frc.robot.subsystems.drivetrain.Drivetrain;
// import frc.robot.subsystems.drivetrain.DrivetrainIO;
// import frc.robot.subsystems.drivetrain.DrivetrainIOSim;
// import frc.robot.subsystems.drivetrain.DrivetrainIOSparkMax;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.IntakeIO;
// import frc.robot.subsystems.intake.IntakeIOSim;
// import frc.robot.subsystems.intake.IntakeIOSparkMax;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.ShooterIO;
// import frc.robot.subsystems.shooter.ShooterIOSim;
// import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.winch.Winch;
import frc.robot.subsystems.winch.WinchIO;
import frc.robot.subsystems.winch.WinchIOSparkMax;
// import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    // Xbox controllers
    public final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
    public final XboxController operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER);

    // Subsystems
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Arm arm;
    private final Shooter shooter;
    private final Vision vision;
    private final Winch winch;

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
    private final DrivingIntake drivingIntake;

    // Named Commands (for autos)
    public NamedCommands scoreSpeaker;
    public NamedCommands runIntake;    

    //start time
    public double startTime = Timer.getFPGATimestamp();

    public RobotContainer() {
        // Creates a real robot.
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOSparkMax());
            intake = new Intake(new IntakeIOSparkMax());
            arm = new Arm(new ArmIOSparkMax());
            shooter = new Shooter(new ShooterIOSparkMax());
            vision = new Vision();
            winch = new Winch(new WinchIOSparkMax());
        }
        // Creates a simulated robot.
        else if (RobotBase.isSimulation()) {
            drivetrain = new Drivetrain(new DrivetrainIOSim());
            arm = new Arm(new ArmIOSim());
            intake = new Intake(new IntakeIOSim());
            shooter = new Shooter(new ShooterIOSim());
            vision = new Vision();
            winch = new Winch(new WinchIO());
        } 
        // Creates a replay robot.
        else {
            drivetrain = new Drivetrain(new DrivetrainIO());
            intake = new Intake(new IntakeIO());
            arm = new Arm(new ArmIO());
            shooter = new Shooter(new ShooterIO());
            vision = new Vision();
            winch = new Winch(new WinchIO());
        }

        teleopDrive = new TeleopDrive(drivetrain, driverController);
        drivetrain.setDefaultCommand(teleopDrive);

        armPID = new ArmPID(arm, ArmConstants.STARTING_POSITION);
        arm.setDefaultCommand(armPID);

        drivingIntake = new DrivingIntake(intake, operatorController);
        intake.setDefaultCommand(drivingIntake);

        configureAuto();
        configureBindings();
    }

    private void configureAuto() {
        autoChooser = new LoggedDashboardChooser<Command>("auto chooser");
        positionChooser = new LoggedDashboardChooser<Pose2d>("position chooser");

        // Named commands for autos.
        NamedCommands.registerCommand("scoreSpeaker", Score.scoreSpeaker(arm, armPID, shooter, intake));
        NamedCommands.registerCommand("scoreAmp", Score.scoreAmp(arm, armPID, shooter, intake));

        // Shouldn't need stopScoring.
        NamedCommands.registerCommand("stopScoring", new InstantCommand(() -> Score.stop(arm, shooter, intake)));
        NamedCommands.registerCommand("runIntake", new InstantCommand(() -> intake.setVoltage(12)));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.setVoltage(0)));
        //NamedCommands.registerCommand("spin180", new ProfiledPIDSetAngle(drivetrain, Units.degreesToRadians(120)));
        NamedCommands.registerCommand("invertPose", new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(drivetrain.getPose().getTranslation(), drivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(180))))));


        // autoChooser.addDefaultOption("2 Note Auto", new PathPlannerAuto("2 Note Auto"));
        //positionChooser.addDefaultOption("Red Amp", PositionConstants.RED_AMP);

        // Placeholders until autos are coded.
        autoChooser.addDefaultOption("2 Note Amp", new PathPlannerAuto("2 Note Amp"));
        autoChooser.addOption("2 Note Stage", new PathPlannerAuto("2 Note Stage"));
        autoChooser.addOption("2 Note Source", new PathPlannerAuto("2 Note Source"));
        autoChooser.addOption("1 Note", new PathPlannerAuto("1 Note"));

        // Characterization routines.
        autoChooser.addOption("Drivetrain Velocity", drivetrain.characterizeVelocity());
        autoChooser.addOption("Drivetrain Acceleration", drivetrain.characterizeAcceleration());

        // Placeholders until positions are configured.
        positionChooser.addOption("Red Amp", PositionConstants.RED_AMP);
        positionChooser.addOption("Red Stage", PositionConstants.RED_STAGE);
        positionChooser.addOption("Red Source", PositionConstants.RED_SOURCE);
        positionChooser.addDefaultOption("Blue Amp", PositionConstants.BLUE_AMP);
        positionChooser.addOption("Blue Stage", PositionConstants.BLUE_STAGE);
        positionChooser.addOption("Blue Source", PositionConstants.BLUE_SOURCE);

        manualIntake = new LoggedDashboardBoolean("manual intake");
        disableArm = new LoggedDashboardBoolean("disable arm");
        resetArmEncoder = new LoggedDashboardBoolean("reset arm encoder");

        PathPlannerLogging.setLogTargetPoseCallback(pose -> {
            Logger.recordOutput("Target Pose", pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback(pose -> {
            Logger.recordOutput("Current Pathplanner Pose", pose);
        });

        PathPlannerLogging.setLogActivePathCallback(path -> {
            Logger.recordOutput("Trajectory", path.toArray(new Pose2d[path.size()]));
        });

        AutoBuilder.configureRamseteRefine(
            new RamseteController(), 
            drivetrain::getPose, 
            drivetrain::getSpeeds, 
            drivetrain::resetOdometry, 
            speeds -> {
                drivetrain.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
            }, 
            new GlobalConfig(3, 3, 3, drivetrain.kinematics), 
            drivetrain
        );

        AstrolabeLogger.targetPoseLogger = pose -> Logger.recordOutput("Target Pose", pose);
        AstrolabeLogger.stateLogger = state -> Logger.recordOutput("Pathing State", state);
        AstrolabeLogger.trajectoryLogger = t -> Logger.recordOutput("Astrolabe Trajectory", t);
        AstrolabeLogger.angleErrorDegreesLogger = error -> Logger.recordOutput("Astrolabe Angle Error", error);
        AstrolabeLogger.distanceErrorLogger = error -> Logger.recordOutput("Astrolabe Distance Error", error);
    }

    private void configureBindings() {
        Trigger driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        Trigger driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        Trigger driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
        Trigger driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
        Trigger driverX = new JoystickButton(driverController, XboxController.Button.kY.value);
        Trigger driverY = new JoystickButton(driverController, XboxController.Button.kY.value);

        // Binds macros for orienting robot turning to driver's dpad.
        // new Trigger(() -> driverController.getPOV() == 0).onTrue(new ProfiledPIDSetAngle(drivetrain, 0));
        // new Trigger(() -> driverController.getPOV() == 45).onTrue(new ProfiledPIDSetAngle(drivetrain, Math.PI / 4));
        // new Trigger(() -> driverController.getPOV() == 90).onTrue(new ProfiledPIDSetAngle(drivetrain, Math.PI / 2));
        // new Trigger(() -> driverController.getPOV() == 135).onTrue(new ProfiledPIDSetAngle(drivetrain, (3 * Math.PI) / 4));
        // new Trigger(() -> driverController.getPOV() == 180).onTrue(new ProfiledPIDSetAngle(drivetrain, Math.PI));
        // new Trigger(() -> driverController.getPOV() == 225).onTrue(new ProfiledPIDSetAngle(drivetrain, (5 * Math.PI) / 4));
        // new Trigger(() -> driverController.getPOV() == 270).onTrue(new ProfiledPIDSetAngle(drivetrain, (3 * Math.PI) / 2));
        // new Trigger(() -> driverController.getPOV() == 315).onTrue(new ProfiledPIDSetAngle(drivetrain, (7 * Math.PI) / 4));

        Trigger operatorLeftStickButton = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
        Trigger operatorRightStickButton = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
        Trigger operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
        Trigger operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
        Trigger operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
        Trigger operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
        Trigger operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
        Trigger operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
        Trigger DPadDown = new Trigger(() -> operatorController.getPOV() == 180);
        Trigger DPadUp = new Trigger(() -> operatorController.getPOV() == 0);

        operatorA.onTrue(Score.ferryNote(arm, armPID, shooter, intake));
        operatorB.whileTrue(Score.scoreAmp(arm, armPID, shooter, intake));
        operatorX.onTrue(new InstantCommand(() -> armPID.setTarget(ArmConstants.STARTING_POSITION)));
        operatorY.whileTrue(Score.scoreSpeaker(arm, armPID, shooter, intake));

        // Left bumper hands off to the shooter, while right bumper reverse-handoffs back to the intake.
        operatorLeftBumper.onTrue(new RaceFeed(shooter, intake).withTimeout(3));
        operatorRightBumper.onTrue(new ReverseFeed(shooter, intake, 1)); // Meters is a complete guess.

        // Climbing commands.
        operatorLeftStickButton.onTrue(new RaiseArm(arm, operatorController, armPID));
        // operatorRightStickButton.onTrue(new Climb(arm, winch, operatorController));

        DPadUp.onTrue(new InstantCommand(() -> armPID.setTarget(ArmConstants.STARTING_POSITION + 1.35)));
        DPadDown.whileTrue(new Climb(arm, winch, operatorController)).and(() -> Timer.getFPGATimestamp() - startTime < 60);
    }

    public void startMatch() {
        drivetrain.resetOdometry(positionChooser.get());
    }

    public Command getAutonomousCommand() {
        //return new ProfiledPIDSetAngle(drivetrain, Math.PI / 2); 
        //return new IntakePieceSpeed(intake);
        //return new PathPlannerAuto("2 Note Source");//new PathPlannerAuto("Backwards Test");//new InstantCommand(() -> armPID.setTarget(Units.degreesToRadians(30)));//
        //return autoChooser.get();
        //return new FollowPath("Source Part A").andThen(new FollowPath("Source Part B"));
        return new FollowTrajectory("Source Part A");
        //return new PathPlannerAuto("Forwards Back");
        //return new PathPlannerAuto("1 Meter Forward");
    }
}
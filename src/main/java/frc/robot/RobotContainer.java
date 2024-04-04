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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.commands.Intake.ShooterPositionFeed;
// import frc.robot.commands.arm.ArmPID;
// import frc.robot.commands.driving.ProfiledPIDSetAngle;
//import frc.robot.commands.driving.ProfiledPIDSetAngle;
// import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.Intake.AltImprovedFeed;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Intake.DrivingIntake;
import frc.robot.commands.Intake.Feed;
import frc.robot.commands.Intake.RaceFeed;
import frc.robot.commands.Intake.ReverseFeed;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.arm.Climb;
// import frc.robot.commands.scoring.Score;
// import frc.robot.commands.arm.Climb;
import frc.robot.commands.arm.RaiseArm;
import frc.robot.commands.driving.DrivePosition;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.scoring.Score;
import frc.robot.commands.shooter.ShooterPID;
import frc.robot.commands.vision.NoteLocalizer;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private final NoteLocalizer noteLocalizer;

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
            vision = new Vision(new VisionIO());//USB());
            winch = new Winch(new WinchIOSparkMax());
        }
        // Creates a simulated robot.
        else if (RobotBase.isSimulation()) {
            drivetrain = new Drivetrain(new DrivetrainIOSim());
            arm = new Arm(new ArmIOSim());
            intake = new Intake(new IntakeIOSim());
            shooter = new Shooter(new ShooterIOSim());
            // vision = new Vision(new VisionIOSim(new Translation2d(4, 4), () -> {
            //     return drivetrain.getPose();
            // }));

            vision = new Vision(new VisionIO());
            winch = new Winch(new WinchIO());
        } 
        // Creates a replay robot.
        else {
            drivetrain = new Drivetrain(new DrivetrainIO());
            intake = new Intake(new IntakeIO());
            arm = new Arm(new ArmIO());
            shooter = new Shooter(new ShooterIO());
            vision = new Vision(new VisionIO());
            winch = new Winch(new WinchIO());
        }

        teleopDrive = new TeleopDrive(drivetrain, driverController);
        drivetrain.setDefaultCommand(teleopDrive);

        armPID = new ArmPID(arm, ArmConstants.STARTING_POSITION);
        arm.setDefaultCommand(armPID);

        drivingIntake = new DrivingIntake(intake, operatorController);
        intake.setDefaultCommand(drivingIntake);

        // noteLocalizer = new NoteLocalizer(vision, drivetrain::getPose);
        // vision.setDefaultCommand(noteLocalizer);

        noteLocalizer = null;

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
        AutoBuilder.configureRamseteRefine(
            new RamseteController(), 
            drivetrain::getPose, 
            drivetrain::getSpeeds, 
            drivetrain::resetOdometry, 
            speeds -> {
                drivetrain.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
            }, 
            new GlobalConfig(3, 3, 3, drivetrain.kinematics), 
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
            drivetrain
        );

        AstrolabeLogger.targetPoseLogger = pose -> Logger.recordOutput("Target Pose", pose);
        AstrolabeLogger.stateLogger = state -> Logger.recordOutput("Pathing State", state);
        AstrolabeLogger.trajectoryLogger = t -> Logger.recordOutput("Astrolabe Trajectory", t);
        AstrolabeLogger.angleErrorDegreesLogger = error -> Logger.recordOutput("Astrolabe Angle Error", error);
        AstrolabeLogger.distanceErrorLogger = error -> Logger.recordOutput("Astrolabe Distance Error", error);

        // Amp autos.

        autoChooser.addDefaultOption("Amp Score:[P, 3]", 
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new InstantCommand(() -> intake.setVoltage(12)))
                .finallyDo(() -> Logger.recordOutput("Astrolabe Pathing", true))
                .andThen(new FollowTrajectory("2 Note Forward A"))
                .finallyDo(() -> Logger.recordOutput("Astrolabe Pathing", false))
                .andThen(new InstantCommand(() -> intake.setVoltage(0)))
                .andThen(new FollowTrajectory("2 Note B Forward"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        autoChooser.addOption("Amp Score:[P, 8]",
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Amp Part C")))
                .andThen(new FollowTrajectory("Amp Part D"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        autoChooser.addOption("Amp Score:[P, 3] Ferry:[8, 7]",
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("2 Note Forward A")))
                .andThen(new FollowTrajectory("2 Note B Forward"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Amp Part C")))
                .andThen(new FollowTrajectory("Amp Part E"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Amp Part F")))
                .andThen(new FollowTrajectory("Amp Part G"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        // Source autos.

        autoChooser.addOption("Source Score:[P, 1]", 
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new InstantCommand(() -> intake.setVoltage(12)))
                .andThen(new FollowTrajectory("Source Part A"))
                .andThen(new InstantCommand(() -> intake.setVoltage(0)))
                .andThen(new FollowTrajectory("Source Part B"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        autoChooser.addOption("Source Score:[P, 1, 4]", 
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new InstantCommand(() -> intake.setVoltage(12)))
                .andThen(new FollowTrajectory("Source Part A"))
                .andThen(new InstantCommand(() -> intake.setVoltage(0)))
                .andThen(new FollowTrajectory("Source Part B"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
                .andThen(new InstantCommand(() -> intake.setVoltage(12)))
                .andThen(new FollowTrajectory("Source Part C"))
                .andThen(new InstantCommand(() -> intake.setVoltage(0)))
                .andThen(new FollowTrajectory("Source Part D"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        autoChooser.addOption("Source Score:[P, 4]",
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Source Part C")))
                .andThen(new FollowTrajectory("Source Part D"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        // Stage autos.

        autoChooser.addOption("Stage Score:[P, 2]", 
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new InstantCommand(() -> intake.setVoltage(12)))
                .andThen(new FollowTrajectory("Stage Part A"))
                .andThen(new InstantCommand(() -> intake.setVoltage(0)))
                .andThen(new FollowTrajectory("Stage Part B"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        autoChooser.addOption("Stage Score:[P, 2, 8]",
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Stage Part A")))
                .andThen(new FollowTrajectory("Stage Part B").beforeStarting(() -> shooter.setVoltage(5), shooter))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Stage Part C")))
                .andThen(new FollowTrajectory("Stage Part D").beforeStarting(() -> shooter.setVoltage(5), shooter))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

        // Misc. autos.
 
        autoChooser.addOption("Score:[P]",
            Score.scoreSpeaker(arm, armPID, shooter, intake)
        );

        autoChooser.addOption("Stage 2 Ferry 7 & 6", 
            Score.scoreSpeaker(arm, armPID, shooter, intake)
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Stage Part A")))
                .andThen(new FollowTrajectory("Stage Part D"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("Stage to 7")))
                .andThen(new FollowTrajectory("7 to ferry"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
                .andThen(new AutoIntake(intake, shooter).raceWith(new FollowTrajectory("7.5 to 6")))
                .andThen(new FollowTrajectory("6 to ferry"))
                .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake))
        );

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
    }

    private void configureBindings() {
        Trigger driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        Trigger driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        Trigger driverLeftTrigger = new JoystickButton(driverController, XboxController.Axis.kLeftTrigger.value);
        Trigger driverRightTrigger = new JoystickButton(driverController, XboxController.Axis.kRightTrigger.value);
        Trigger driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
        Trigger driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
        Trigger driverX = new JoystickButton(driverController, XboxController.Button.kY.value);
        Trigger driverY = new JoystickButton(driverController, XboxController.Button.kY.value);

        driverLeftBumper.onTrue(new InstantCommand(() -> teleopDrive.setPrecisionDrive(true)));
        driverLeftBumper.onFalse(new InstantCommand(() -> teleopDrive.setPrecisionDrive(false)));
        driverRightBumper.onTrue(new InstantCommand(() -> teleopDrive.setPrecisionDrive(true)));
        driverRightBumper.onFalse(new InstantCommand(() -> teleopDrive.setPrecisionDrive(false)));

        // driverB.whileTrue(
        //     new InstantCommand(() -> intake.setVoltage(12)).andThen(new DrivePosition(drivetrain, noteLocalizer::getNotePosition)).finallyDo(() -> intake.setVoltage(0))
        // );

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

        //operatorA.onTrue(Score.ferryNote(arm, armPID, shooter, intake));
        operatorA.whileTrue(new ShooterPID(shooter, ShooterConstants.SPEAKER_RPS));
        operatorB.whileTrue(Score.scoreAmp(arm, armPID, shooter, intake));
        operatorX.onTrue(new InstantCommand(() -> armPID.setTarget(ArmConstants.STARTING_POSITION)));
        operatorY.whileTrue(Score.scoreSpeaker(arm, armPID, shooter, intake));

        // Left bumper hands off to the shooter, while right bumper reverse-handoffs back to the intake.
        operatorLeftBumper.whileTrue(new RaceFeed(shooter, intake));
        operatorRightBumper.whileTrue(new ReverseFeed(shooter, intake, Units.inchesToMeters(14))); // Meters is a complete guess.

        // Climbing commands.
        // operatorLeftStickButton.toggleOnTrue(new RaiseArm(arm, operatorController, armPID));
        // operatorRightStickButton.onTrue(new Climb(arm, winch, operatorController));

        DPadUp.onTrue(new RaiseArm(arm, operatorController, armPID));
        DPadDown.onTrue(new Climb(arm, winch, operatorController).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
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
        // return Score.scoreSpeaker(arm, armPID, shooter, intake)
        //     .andThen(new InstantCommand(() -> intake.setVoltage(12)))
        //     .andThen(new FollowTrajectory("Source Part A"))
        //     .andThen(new InstantCommand(() -> intake.setVoltage(0)))
        //     .andThen(new FollowTrajectory("Source Part B"))
        //     .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake));
        // return Score.scoreSpeaker(arm, armPID, shooter, intake)
        //     .andThen(new InstantCommand(() -> intake.setVoltage(12)))
        //     .finallyDo(() -> Logger.recordOutput("Astrolabe Pathing", true))
        //     .andThen(new FollowTrajectory("2 Note Forward A"))
        //     .finallyDo(() -> Logger.recordOutput("Astrolabe Pathing", false))
        //     .andThen(new InstantCommand(() -> intake.setVoltage(0)))
        //     .andThen(new FollowTrajectory("2 Note B Forward"))
        //     .andThen(Score.scoreSpeaker(arm, armPID, shooter, intake));
        //return new PathPlannerAuto("Forwards Back");
        //return new PathPlannerAuto("1 Meter Forward");

        return autoChooser.get();
    }
}
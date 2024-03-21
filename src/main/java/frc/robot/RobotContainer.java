// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import frc.robot.commands.manipulatorStates.BringElevatorDown;
import frc.robot.commands.manipulatorStates.FillBox;
import frc.robot.commands.manipulatorStates.FillChute;
import frc.robot.commands.manipulatorStates.PrepBoxForSpeaker;
import frc.robot.commands.manipulatorStates.RaiseElevator;
import frc.robot.commands.manipulatorStates.ScoreInAmp;
import frc.robot.commands.manipulatorStates.ScoreSpeaker;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ManipulatorSubsystem m_robotManipulator = new ManipulatorSubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(1);

    private boolean m_fieldRelative = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                m_fieldRelative, true),
                        m_robotDrive));

        //manipulator idle is the default
        m_robotManipulator.setDefaultCommand(new RunCommand(() -> {
//            m_robotManipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
            m_robotManipulator.intakeFrontMotorSpeed(ManipulatorConstants.kIntakeFrontOut);
            m_robotManipulator.intakeRearMotorSpeed(ManipulatorConstants.kIntakeRearOut);
            
            m_robotManipulator.chuteMotorSpeed(ManipulatorConstants.kChuteOff);

            m_robotManipulator.boxLeftMotorSpeed(ManipulatorConstants.kBoxLeftOff);
            m_robotManipulator.boxRightMotorSpeed(ManipulatorConstants.kBoxRightOff);

            SmartDashboard.putString("command:", "idle");
        }, m_robotManipulator));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        new JoystickButton(m_driverController, OIConstants.kRightBumper)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

        new JoystickButton(m_driverController, OIConstants.kButtonY)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));

        new JoystickButton(m_driverController, OIConstants.kButtonX)
            .toggleOnTrue(new InstantCommand(
            () -> {
                m_fieldRelative = !m_fieldRelative;
                SmartDashboard.putBoolean("field relative", m_fieldRelative);
            }
            , m_robotDrive));


        //manipulator

        Timer stateTimer = new Timer();

        //intake command group
        SequentialCommandGroup intakeRequest = new SequentialCommandGroup(); 
        intakeRequest.addCommands(new FillChute(m_robotManipulator));
        intakeRequest.addRequirements(m_robotManipulator);

        SequentialCommandGroup speakerRequest = new SequentialCommandGroup();
        speakerRequest.addCommands(new PrepBoxForSpeaker(stateTimer, m_robotManipulator));
        speakerRequest.addCommands(new ScoreSpeaker(m_robotManipulator));
        speakerRequest.addRequirements(m_robotManipulator);

        SequentialCommandGroup ampRequest = new SequentialCommandGroup();
        ampRequest.addCommands(new FillBox(m_robotManipulator));
        ampRequest.addCommands(new RaiseElevator(m_robotManipulator, ManipulatorConstants.kElevatorAmpPos)); 
        ampRequest.addCommands(new ScoreInAmp(stateTimer, m_robotManipulator));
        ampRequest.addCommands(new BringElevatorDown(m_robotManipulator));

        ampRequest.addRequirements(m_robotManipulator);
        
        //intake request
        new JoystickButton(m_operatorController, OIConstants.kButtonA)
            .onTrue(intakeRequest);
 

        ///amp request
        new JoystickButton(m_operatorController, OIConstants.kButtonB)
            .onTrue(ampRequest);

        //speaker request
        new JoystickButton(m_operatorController, OIConstants.kButtonX)
            .onTrue(speakerRequest);

        //reset boxFull and chuteFull
        new JoystickButton(m_operatorController, OIConstants.kButtonB)
            .onTrue(new InstantCommand(() -> {
                m_robotManipulator.boxSetState(false);
                m_robotManipulator.chuteSetState(false);
            }));

        //emergancy stop all commands
        new JoystickButton(m_operatorController, OIConstants.kButtonStart)
            .onTrue(new InstantCommand(() -> {
                CommandScheduler.getInstance().cancelAll();
            }));
        
        new JoystickButton(m_driverController, OIConstants.kButtonStart)
            .onTrue(new InstantCommand(() -> {
                CommandScheduler.getInstance().cancelAll();
            }));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        Trajectory straightAhead = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(Units.feetToMeters(-8), 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                straightAhead,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(straightAhead.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

    }
}

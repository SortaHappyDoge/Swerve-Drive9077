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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.Console;
import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final AutonomousCommands autonomousCommands = new AutonomousCommands(m_robotDrive); // Pass m_robotDrive
  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public static Field2d m_field = new Field2d();
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*((m_driverController.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*((m_driverController.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRawAxis(2))*0.7, OIConstants.kDriveDeadband*3),
                true),
            m_robotDrive));
        
    SmartDashboard.putData("Field", m_field);
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        m_field.setRobotPose(pose);
    });
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        m_field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        m_field.getObject("path").setPoses(poses);
    });
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, 12)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(), 
            m_robotDrive));
    new JoystickButton(m_driverController, 3)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.toggleFieldRelative(),
            m_robotDrive));
    new JoystickButton(m_driverController, 1)
        .onTrue(new InstantCommand(
            () -> autonomousCommands.ResetRotationPID(),
            m_robotDrive));
    new JoystickButton(m_driverController, 1)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.Target(m_driverController),
            m_robotDrive));
    new JoystickButton(m_driverController, 2)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.Rotate90(m_driverController),
            m_robotDrive));
    new JoystickButton(m_driverController, 11)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.resetOdometry(new Pose2d()),
             m_robotDrive));
    new JoystickButton(m_driverController, 4)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.m_elevator.setDesiredHeight(0),
             m_robotDrive));
    new JoystickButton(m_driverController, 5)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.m_elevator.setDesiredHeight(.6),
             m_robotDrive));
    new JoystickButton(m_driverController, 6)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.m_elevator.setDesiredHeight(1.34),
             m_robotDrive));
    new JoystickButton(m_driverController, 7)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.m_armSubsystem.loadCoral(),
            m_robotDrive));  
    new JoystickButton(m_driverController, 7)
        .onTrue(new RunCommand(
            () -> m_robotDrive.m_armSubsystem.loadCoralManual(0.2),
            m_robotDrive));
    new JoystickButton(m_driverController, 10)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.m_armSubsystem.armTest(1),
            m_robotDrive)); 
    new JoystickButton(m_driverController, 9)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.m_armSubsystem.armTest(-1),
            m_robotDrive)); 
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //PathPlannerPath path = PathPlannerPath.fromPathFile("ID 12 Source.path");
    try {
        return AutoBuilder.pathfindToPose(m_robotDrive.m_autonCmds.autonDestinations[0], AutoConstants.kPathfindingConstraints).andThen(AutoBuilder.pathfindToPose(m_robotDrive.m_autonCmds.autonDestinations[2], AutoConstants.kPathfindingConstraints)).andThen(AutoBuilder.pathfindToPose(m_robotDrive.m_autonCmds.autonDestinations[1], AutoConstants.kPathfindingConstraints));
        //return AutoBuilder.followPath(auto);
        
    } catch (Exception e) {
        System.out.println(e);
        return null;
    }
}
}

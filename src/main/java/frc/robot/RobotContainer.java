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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.Console;
import java.nio.file.Path;
import java.security.AllPermission;
import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
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
  public XboxController m_driverController = new XboxController(1);
  public XboxController m_elevatorController = new XboxController(0);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public static Field2d nav_field = new Field2d();
  public static Field2d m_field = new Field2d();
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();
        
    // Configure default commands
    /*m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getRightY()*((m_elevatorController.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*((m_elevatorController.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2))*0.5, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));*/
        
    SmartDashboard.putData("Robot Field", m_field);
    SmartDashboard.putData("Pathplanner Field", nav_field);
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        nav_field.setRobotPose(pose);
    });
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        nav_field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        nav_field.getObject("path").setPoses(poses);
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
    // Driver 0 - Driver 1 switch
    new JoystickButton(m_elevatorController, 1)
        .onTrue(new InstantCommand(() -> m_robotDrive.setFieldRelative(false), m_robotDrive))
        .onFalse(new InstantCommand(() -> m_robotDrive.setFieldRelative(true), m_robotDrive));
    //



    // Driver 0
    new JoystickButton(m_driverController, 4)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    new POVButton(m_driverController, 180)
        .onTrue(m_robotDrive.m_armSubsystem.loadCoral(-0.3));
    new POVButton(m_driverController, 0)
        .onTrue(new InstantCommand(()-> m_robotDrive.m_armSubsystem.loadCoralManual(0.3)))
        .onFalse(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(0)));
        
    new POVButton(m_driverController, 90)
        .onTrue(m_robotDrive.m_autonCmds.pathfindToReef(12, false, true));
    new POVButton(m_driverController, 270)
        .onTrue(m_robotDrive.m_autonCmds.pathfindToReef(13, false, true));

    new JoystickButton(m_driverController, 2)
        .onTrue(new InstantCommand(() -> m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kBallHeights[0])))
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[5])));
    
    //


    
    // Elevator Height Setting Driver 1
    new JoystickButton(m_elevatorController, 3)
        .onTrue(m_robotDrive.m_autonCmds.raiseElevatorConditionless(0));
    new JoystickButton(m_elevatorController, 4)
        .onTrue(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1));
    new JoystickButton(m_elevatorController, 5)
        .onTrue(m_robotDrive.m_autonCmds.raiseElevatorConditionless(2));
    new JoystickButton(m_elevatorController, 6)
        .onTrue(m_robotDrive.m_autonCmds.raiseElevatorConditionless(3));
        
    /*
    new JoystickButton(m_elevatorController, 11)
        .onTrue(new InstantCommand(() -> m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kBallHeights[1])))
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[4])))
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(0.4)));
    new JoystickButton(m_elevatorController, 12)
        .onTrue(new InstantCommand(() -> m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kBallHeights[2])))
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[4])))
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(0.4)));
    */
    //
    
        
        
    // Arm Roller Settings Driver 1
    new JoystickButton(m_elevatorController, 10)     // Roll coral back / Intake Ball
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(0.3)));
        //.onFalse(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(0)));
    new JoystickButton(m_elevatorController, 2)     // Intake Coral / Drop Ball
        .onTrue(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(-0.3)))
        .onFalse(new InstantCommand(() -> m_robotDrive.m_armSubsystem.loadCoralManual(0)));
    new JoystickButton(m_elevatorController, 8)
        .onTrue(m_robotDrive.m_armSubsystem.loadCoral(-0.3));
    //


}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    boolean isBlu;
    try{
        isBlu = DriverStation.getAlliance().get() == Alliance.Blue;
    }
    catch(Exception e){
        isBlu = true;
    }
    isBlu = true;
    return 
        new WaitCommand(0)
        // Left 2 coral closer end
        
        /*.andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(19, false, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(13, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(19, true, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(13, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        

        // Left 2 one far one closer end
        /*
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(20, true, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(13, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(19, true, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(13, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        */
        
        // Right 2 coral closer end
        /*
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(17, true, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(12, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(17, false, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(12, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        */

        // Right 2 one far one closer end
        /*
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(22, true, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(12, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(17, false, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(12, false, isBlu))
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        */

        // Middle
        /*
        .andThen(m_robotDrive.m_armSubsystem.loadCoral(-0.3))
        .andThen(m_robotDrive.m_autonCmds.raiseElevatorConditionless(1))
        .andThen(m_robotDrive.m_autonCmds.pathfindToReef(21, false, isBlu))
        .andThen(m_robotDrive.m_autonCmds.scoreCoral(3))
        */
        ; 
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;

//import edu.wpi.first.wpilibj2.command.RunCommand;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private AutonomousCommands autonCmds;
  private RobotContainer m_robotContainer;
  public boolean autonHasStarted = false;
  public int driveMultiplier = -1; // -1 for blue 1 for red
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    /*enableLiveWindowInTest(true);
    LiveWindow.setEnabled(true);
    LiveWindow.enableAllTelemetry();*/

    m_robotContainer = new RobotContainer();
    autonCmds = m_robotContainer.m_robotDrive.m_autonCmds;
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    //m_robotContainer.m_robotDrive.setStartingPose();
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if(!autonHasStarted){
      m_robotContainer.m_robotDrive.setStartingPose();
      if(DriverStation.getAlliance().get() == Alliance.Blue) driveMultiplier = -1;
      else driveMultiplier = 1;
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //m_robotContainer.m_robotDrive.setStartingPose();
    ////autonCmds.autonState = 1;
    autonHasStarted = true;
    if(m_autonomousCommand != null){
      m_autonomousCommand.schedule();
    }
    //m_robotContainer.autonomousCommands.scoreCoral(3);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //lastAutonPose = new Pose2d(lastAutonPose.getTranslation(), lastAutonPose.getRotation().rotateBy(Rotation2d.k180deg));
    /*if(lastAutonPose != null)
    m_robotContainer.m_robotDrive.resetPose(lastAutonPose);*/
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Driver 0 Movement
    /*if(!m_robotContainer.m_elevatorController.getRawButton(1) && !CommandScheduler.getInstance().isScheduled(m_robotContainer.m_robotDrive.getCurrentCommand()))
    new RunCommand(
            () -> m_robotContainer.m_robotDrive.drive(
                driveMultiplier*MathUtil.applyDeadband(m_robotContainer.m_driverController.getRightY()*((m_robotContainer.m_elevatorController.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
                driveMultiplier*MathUtil.applyDeadband(m_robotContainer.m_driverController.getRightX()*((m_robotContainer.m_elevatorController.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_robotContainer.m_driverController.getRawAxis(3) - m_robotContainer.m_driverController.getRawAxis(2))*0.5, OIConstants.kDriveDeadband),
                true),
            m_robotContainer.m_robotDrive).schedule();
    else if(!CommandScheduler.getInstance().isScheduled(m_robotContainer.m_robotDrive.getCurrentCommand()))*/
    m_robotContainer.m_robotDrive.setFieldRelative(false);
    new RunCommand(
            () -> m_robotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(m_robotContainer.m_elevatorController.getLeftY()*0.3/*((m_robotContainer.m_elevatorController.getRawAxis(3)*(-1)+1)/2)*/, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_robotContainer.m_elevatorController.getLeftX()*0.3/*((m_robotContainer.m_elevatorController.getRawAxis(3)*(-1)+1)/2)*/, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_robotContainer.m_elevatorController.getRawAxis(2))*0.3, OIConstants.kDriveDeadband*3),
                true),
            m_robotContainer.m_robotDrive).schedule();
    //


    // Driver 1 Pathfinding
    /*if(m_robotContainer.m_elevatorController.getRawButtonPressed(8)){
      try {
        Command cmd = m_robotContainer.m_robotDrive.m_autonCmds.selectReef(true, m_robotContainer.m_elevatorController, m_robotContainer.m_elevatorController.getPOV());
        if(cmd != null)
        cmd.schedule();
      } catch (Exception e) {
        System.out.println(e);
      }
    }
    if(m_robotContainer.m_elevatorController.getRawButtonPressed(7)){
      try {
        Command cmd = m_robotContainer.m_robotDrive.m_autonCmds.selectReef(false, m_robotContainer.m_elevatorController, m_robotContainer.m_elevatorController.getPOV());
        if(cmd != null)
        cmd.schedule();
      } catch (Exception e) {
        System.out.println(e);
      }
    }


    if(m_robotContainer.m_elevatorController.getRawButtonPressed(12)){
      try {
        Command cmd = m_robotContainer.m_robotDrive.m_autonCmds.selectBall(m_robotContainer.m_elevatorController, m_robotContainer.m_elevatorController.getPOV());
        System.out.print("created ball command");
        if(cmd != null)
        cmd.schedule();
        System.out.print("scheduled ball command");
      } catch (Exception e) {
        System.out.println(e);
      }
    }
    //
    */

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}

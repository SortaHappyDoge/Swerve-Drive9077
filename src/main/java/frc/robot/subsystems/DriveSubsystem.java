// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonomousCommands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.chrono.ThaiBuddhistChronology;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  RobotConfig config;

  // Configure Pigeon 2.0 in Tuner X for ID, default roborio canbus "rio"
  private final Pigeon2 pigeon2 = new Pigeon2(0, "rio");  
  
  public AutonomousCommands m_autonCmds; // Pass m_robotDrive
  public ArmSubsystem m_armSubsystem;
  public ElevatorSubsystem m_elevator;

  private boolean fieldRelative = true;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  public Pose2d m_robotPose; // Pose of the robot compared to the field 
  private SwerveDrivePoseEstimator m_poseEstimator;
  boolean zeroPoseWithLL = true;
  boolean rejectLLupdate = false;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  SwerveModuleState[] m_moduleStates = new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(0),
      new SwerveModulePosition[]{
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }
    );

  public Pose2d[] aprilTagPositions = {
    new Pose2d(new Translation2d(16.7072, 0.6553), new Rotation2d()),
    new Pose2d(new Translation2d(16.7072, 7.3965), new Rotation2d()),
    new Pose2d(new Translation2d(11.5608, 8.0553), new Rotation2d()),
    new Pose2d(new Translation2d(9.2761, 6.1377), new Rotation2d()),
    new Pose2d(new Translation2d(9.2761, 1.9149), new Rotation2d()),
    new Pose2d(new Translation2d(13.4745, 3.3063), new Rotation2d(120)),
    new Pose2d(new Translation2d(13.8906, 4.0259), new Rotation2d(180)),
    new Pose2d(new Translation2d(13.4745, 4.7455), new Rotation2d(-120)),
    new Pose2d(new Translation2d(12.6433, 4.7455), new Rotation2d(-60)),
    new Pose2d(new Translation2d(12.2273, 4.0259), new Rotation2d(0)),
    new Pose2d(new Translation2d(12.6433, 3.3063), new Rotation2d(60)),
    new Pose2d(new Translation2d(0.8514, 0.6553), new Rotation2d()),
    new Pose2d(new Translation2d(0.8514, 7.3965), new Rotation2d()),
    new Pose2d(new Translation2d(8.2773, 6.1377), new Rotation2d()),
    new Pose2d(new Translation2d(8.2773, 1.9149), new Rotation2d()),
    new Pose2d(new Translation2d(5.9876, -0.0038), new Rotation2d()),
    new Pose2d(new Translation2d(4.0739, 3.3063), new Rotation2d(60)),
    new Pose2d(new Translation2d(3.6576, 4.0259), new Rotation2d(0)),
    new Pose2d(new Translation2d(4.0739, 4.7455), new Rotation2d(-60)),
    new Pose2d(new Translation2d(4.9049, 4.7455), new Rotation2d(-120)),
    new Pose2d(new Translation2d(5.3111, 4.0259), new Rotation2d(180)),
    new Pose2d(new Translation2d(4.9049, 3.3063), new Rotation2d(120))
};
  public int[] reefAprilTags = {6,7,8,9,10,11,17,18,19,20,21,22};

  public int targetAprilTag = -31;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    m_autonCmds = new AutonomousCommands(this);
    m_armSubsystem = new ArmSubsystem(m_autonCmds, m_elevator);
    m_elevator = new ElevatorSubsystem(m_armSubsystem);

    if(LimelightHelpers.getTV("limelight") && zeroPoseWithLL){
      LimelightHelpers.SetRobotOrientation("limelight", 
      pigeon2.getYaw().getValueAsDouble(), 0, 
      0, 0, 
      0, 0);

      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
    }else{
      m_robotPose = new Pose2d(1, 2, new Rotation2d(m_currentRotation));
    }

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, 
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[]{
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      m_robotPose);

    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    
    configureAutoBuilder();
        //initiateDashboard();
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation("limelight", 
    pigeon2.getYaw().getValueAsDouble(), /*pigeon2.getAngularVelocityYWorld().getValueAsDouble()*/0, 
    0, 0, 
    0, 0);
   
    // Update the odometry in the periodic block
    m_moduleStates = new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()), // edit for pigeon
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    m_poseEstimator.update(
      Rotation2d.fromDegrees(getHeading()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }
    );

    m_robotPose = getPose();
    //updateDashboardValues();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()), // edit for pigeon
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble())) // edit for pigeon
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void toggleFieldRelative(){
    fieldRelative = !fieldRelative;
  }

  public void Target(XboxController controller){
    int i = 1;
    double lowest = 100000;
    int lowestID = -1;
    Pose2d estimate = m_poseEstimator.getEstimatedPosition();

    for (Pose2d pos : aprilTagPositions) {
      double dist = Math.sqrt(Math.pow((pos.getX() - estimate.getX()), 2) + Math.pow((pos.getY() - estimate.getY()), 2));
      if(dist < lowest && Arrays.asList(reefAprilTags).contains(i)){
        lowest = dist;
        lowestID = i;
      }
      i++;
    }
    if(lowest < 100000) m_autonCmds.RotateToAngle(aprilTagPositions[i].getRotation().getDegrees(), controller, AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);
  }

  public void TrackTargetY(XboxController controller){
    if(!LimelightHelpers.getTV("limelight")){ m_autonCmds.RotateToAngle(0, controller, AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD); }
    else{ m_autonCmds.RotateToAngle(LimelightHelpers.getTX("limelight"), controller, AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);}
  }

  public void Rotate90(XboxController controller){
    m_autonCmds.RotateToAngle(90-getHeading(), controller, AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);
  }

  public void rotateArm(Rotation2d angle){
    m_armSubsystem.rotateArmTo(angle);
  }
  
  void addVisionMeasurement(){
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(getTurnRate()) > 720 || mt2.tagCount == 0) rejectLLupdate = true;
    else rejectLLupdate = false;
    if(!rejectLLupdate){
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
    }
  } 
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStatesRobotRelative(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredStateRobotRelative(desiredStates[0]);
    m_frontRight.setDesiredStateRobotRelative(desiredStates[1]);
    m_rearLeft.setDesiredStateRobotRelative(desiredStates[2]);
    m_rearRight.setDesiredStateRobotRelative(desiredStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_moduleStates);
  }
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    setModuleStatesRobotRelative(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon2.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return MathUtil.inputModulus(pigeon2.getYaw().getValueAsDouble(), -180, 180); // edit for pigeon
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return pigeon2.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


  public int getTargetID(){
    return (int)LimelightHelpers.getLimelightNTTableEntry("limelight", "tid").getInteger(-31);
  }

  public void initiateDashboard(){
    SmartDashboard.putData("Swerve",
      builder -> {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty(
              "Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () ->m_frontRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Rear Left Angle", () -> m_rearLeft.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Rear Left Velocity", () -> m_rearLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Rear Right Angle", () -> m_rearRight.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Rear Right Velocity", () -> m_rearRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> Rotation2d.fromDegrees(getHeading()).getRadians(), null);
      }
    );
  }
  public void updateDashboardValues(){

  }

  public void configureAutoBuilder(){
    AutoBuilder.configure(
      () -> m_poseEstimator.getEstimatedPosition(),
      (pose) -> m_poseEstimator.resetPose(pose), 
      () -> getChassisSpeeds(),
      (speeds, feedforwards) -> setChassisSpeeds(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(1, 0),
        new PIDConstants(1, 0)), 
      config, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
  }

}

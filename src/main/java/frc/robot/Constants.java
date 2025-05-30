// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.DecimalFormat;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = .65;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = .65;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kPigeon2CanId = 14;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final DecimalFormat decimalFormat = new DecimalFormat("####0.0000");
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.076;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kElevatorMotorReduction = 6; // This reduction value is after the cartridges and before the movable pulley (only moveable pulley countsss)
    public static final double kElevatorSprocketPitchDiameter = 1.76*2.54/100; // inches to m
    public static final double kElevatorSprocketCircumference = kElevatorSprocketPitchDiameter * Math.PI;

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final double kElevatorMotorSpeedMultiplier = 0.8; // between 0 and 1
    public static final double kElevatorMotorSpeedRampRate = 0.3; // the time (in seconds) for the motor to ramp up from 0% speed to 100%
    public static final double kArmMotorSpeedMultiplier = 0.4; // between 0 and 1 0.44
    public static final double kArmMotorSpeedRampRate = 0.1; // the time (in seconds) for the motor to ramp up from 0% speed to 100%

    public static final int coralQTREmitter = 0;
    public static final int[] coralQTRPins = {1,2,3,4};
    public static final double[] coralQTRErrorMultipliers = {1, 0.5, 0.2, 0, -0.2};

    public static final double kArmMotorReduction = 1/62.5;
    public static final double kRollerMotorReduction = 1/2.5;

    public static final int kElevatorSparkMax0CanID = 10;
    public static final int kElevatorSparkMax1CanID = 11;
    public static final int kArmSparkMaxCanID = 12;
    public static final int kRollerSparkMaxCanID = 13;

    public static final double kElevatorEncoderPositionFactor = (2 * Math.PI);
    public static final double kElevatorEncoderVelocityFactor = kElevatorEncoderPositionFactor / 60;
    public static final double kElevatorHeightLimit = 1.4;
    public static final double kElevatorMinimumPoweredHeight = 0.02;  // In meters
    public static final double kArmMinimumPoweredRotation = 5;  // In degrees


    public static final double kEncoderResetThreshold = 150;  // Some large value that the elevator shaft cant rotate in less then the periodic period

    public static final double kArmEncoderPositionFactor = (2 * Math.PI);
    public static final double kArmEncoderVelocityFactor = kArmEncoderPositionFactor / 60.0;
    public static final double kArmSafeStandoffRotation = 10;
    public static final double[] kArmMinBlockedMaxRotations = {0, 15+kArmSafeStandoffRotation, 300}; // The first and last values indicate min/max arm rotation values respectively

    // All of these need recalculations
    public static final double kArmBlockoffHeightBaseStage = 0.115;
    public static final double kArmBlockerHeightBaseStage = 0.05;
    public static final double kArmBlockoffHeightFistStage = 0.230;
    public static final double kArmBlockerHeightFirstStage = 0.05;

    public static final double kArmBlockoffTolerance = 0.05;

    public static final double[] kArmBlockoffRegions = {
      0,
      kArmBlockoffHeightBaseStage-kArmBlockoffTolerance, 
      kArmBlockoffHeightBaseStage+kArmBlockerHeightBaseStage+kArmBlockoffTolerance,
      0 -kArmBlockoffTolerance, // for base stage bottom height
      0 +kArmBlockoffTolerance, //for base stage top height
      kElevatorHeightLimit
    };
    //

    public static final double kRollerEncoderPositionFactor = (2 * Math.PI) * kRollerMotorReduction;
    public static final double kRollerEncoderVelocityFactor = kRollerEncoderPositionFactor / 60.0;
    public static final double kRollerWheelFreeSpeedRPS = (NeoMotorConstants.kFreeSpeedRpm * kRollerMotorReduction) / 60;
    public static final double kMaxRollerSpeedRPM = NeoMotorConstants.kFreeSpeedRpm * -0.2;

    public static final double[] kElevatorPIDUp = {7, 0, 0.7}; // PID settings for the elevator
    public static final double[] kElevatorPIDDown = {1.25, 0, 0.05}; // PID settings for the elevator
    public static final double kElevatorIZone = 0;
    public static final double kElevatorPIDTolerance = 0.01; // PID tolerance in cm error
    public static final double kElevatorAutonomousTolerance = 0.01; // Tolerance for when the coral is ready to drop
    public static final double[] kArmPID = {0.01, 0, 0.0004}; // PID settings for the arm rotation
    public static final double kArmPIDTolerance = 0.5; // PID tolerance in cm error
    public static final double kArmAutonomousTolerance = 3; // Tolerance for when the coral is ready to drop
    public static final double[] kRollerPID = {0.0001, 0, 0, 1 / kRollerWheelFreeSpeedRPS};  // PIDF settings for the arm rollers
    public static final double[] qtrPID = {0, 0, 0};  // PID settings for the QTR sensor reads
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5;
    public static final double kDXController = 0;
    //public static final double kPYController = 1;
    public static final double kPThetaController = 5;

    public static final double kRotationP = 0.014;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0.0018;
    public static final double kRotationIStart = 2;
    public static final boolean limitI = false;    

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared,
       kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double[] kReefHeights = {0, 0.285, 0.67, 1.33, 0.15}; // L1, L2, L3, L4 and Ready Stance respectively (in meters) (for elevator)
    public static final double[] kBallHeights = {0.25, 0.63, 1}; // Lower and higher algae respectively (in meters) (for elevator)
    public static final double kHeightTolerance = 0.03;
    public static final double[] kReefAngles = {ModuleConstants.kArmMinBlockedMaxRotations[0], ModuleConstants.kArmMinBlockedMaxRotations[1], ModuleConstants.kArmMinBlockedMaxRotations[1], ModuleConstants.kArmMinBlockedMaxRotations[1] + 10, 180, 200}; // L1, L2, L3 , L4 and ball respectively (in degrees) (for arm)


    // x min 0 max 17.54,y min 0 max 0.805
    public static final Pose2d[] kStartingPoses = {
      new Pose2d(7.576,6.5,Rotation2d.fromDegrees(-180)),     // Start 0
      new Pose2d(7.576,4,Rotation2d.fromDegrees(-180)),     // Start 1
      new Pose2d(7.576,1.5,Rotation2d.fromDegrees(-180)),     // Start 2

      new Pose2d(9.964,1.5,Rotation2d.fromDegrees(0)),// Start 3
      new Pose2d(9.964,4,Rotation2d.fromDegrees(0)),// Start 4
      new Pose2d(9.964,6.5,Rotation2d.fromDegrees(0)) // Start 5
    };

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}

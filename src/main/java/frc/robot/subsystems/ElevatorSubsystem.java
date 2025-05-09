package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    ArmSubsystem m_armSubsystem;
    
    public boolean manualControl = false;
    public boolean armBlocked = true;
    public boolean doMove = false;
    public boolean atTarget = false;

    SparkMax m_elevatorSparkMax0;
    SparkMax m_elevatorSparkMax1;

    SparkMaxConfig kElevatorSparkMaxConfig;
    PIDController m_elevatorPIDController  = new PIDController(0, 0, 0);
    PIDController m_elevatorPIDControllerDown  = new PIDController(0, 0, 0);
    SparkAbsoluteEncoder m_elevatorAbsoluteEncoder;

    double lastEncoderPosition;
    double encoderPosition;
    double encoderRotations;
    double startZero;
    public double elevatorPosition; // height in meters
    public double desiredElevatorPosition = 0;
    

    public ElevatorSubsystem(ArmSubsystem armSubsystem){
        this.m_armSubsystem = armSubsystem;

        m_elevatorSparkMax0 = new SparkMax(ModuleConstants.kElevatorSparkMax0CanID, MotorType.kBrushless);
        m_elevatorSparkMax1 = new SparkMax(ModuleConstants.kElevatorSparkMax1CanID, MotorType.kBrushless);

        kElevatorSparkMaxConfig = new SparkMaxConfig();
        kElevatorSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        kElevatorSparkMaxConfig.encoder
            .positionConversionFactor(ModuleConstants.kElevatorEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kElevatorEncoderVelocityFactor);
        kElevatorSparkMaxConfig.absoluteEncoder
            .positionConversionFactor(ModuleConstants.kElevatorEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kElevatorEncoderVelocityFactor)
            .inverted(false);
        kElevatorSparkMaxConfig.openLoopRampRate(ModuleConstants.kElevatorMotorSpeedRampRate);

        m_elevatorSparkMax0.configure(kElevatorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kElevatorSparkMaxConfig.follow(10, true);
        m_elevatorSparkMax1.configure(kElevatorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //m_elevatorPIDController = m_elevatorSparkMax0.getClosedLoopController();
        m_elevatorAbsoluteEncoder = m_elevatorSparkMax0.getAbsoluteEncoder();
        lastEncoderPosition = m_elevatorAbsoluteEncoder.getPosition();
        encoderRotations = 0;

        m_elevatorPIDController.setPID(ModuleConstants.kElevatorPIDUp[0], ModuleConstants.kElevatorPIDUp[1], ModuleConstants.kElevatorPIDUp[2]);
        m_elevatorPIDController.setTolerance(ModuleConstants.kElevatorPIDTolerance);
        
        m_elevatorPIDControllerDown.setPID(ModuleConstants.kElevatorPIDDown[0], ModuleConstants.kElevatorPIDDown[1], ModuleConstants.kElevatorPIDDown[2]);
        m_elevatorPIDController.setTolerance(ModuleConstants.kElevatorPIDTolerance);
        
        
        initiateDashboard();
    }


    @Override
    public void periodic(){
        elevatorPosition = getElevatorPosition();
        //System.out.println(elevatorPosition);
        double absoluteElevatorPosition = getElevatorPositionAbsolute();
        armBlocked = !m_armSubsystem.armStandoffCheck();
        if (armBlocked && manualControl) doMove = false;
        else doMove = true;
        moveElevatorTo(desiredElevatorPosition);

        if(elevatorPosition > 0.25 && elevatorPosition > desiredElevatorPosition-AutoConstants.kHeightTolerance) atTarget = true;
        else atTarget = false;


        updateDashboardValues();
    }

    // In meters
    public void moveElevator(double m){
        moveElevatorTo(elevatorPosition + m);
    }
    public void moveElevatorTo(double m){
        //m_armSubsystem.doRotate = false;
        m = MathUtil.clamp(m, 0, ModuleConstants.kElevatorHeightLimit);
        

        if(desiredElevatorPosition > 0.05 || elevatorPosition > 0.05){
            m_armSubsystem.armMinMaxRotation = new double[]{ModuleConstants.kArmMinBlockedMaxRotations[1], ModuleConstants.kArmMinBlockedMaxRotations[2]};
        }
        else{
            m_armSubsystem.armMinMaxRotation = new double[]{ModuleConstants.kArmMinBlockedMaxRotations[0], ModuleConstants.kArmMinBlockedMaxRotations[2]};
        }
        if(m_armSubsystem.getArmAngle() < ModuleConstants.kArmMinBlockedMaxRotations[1] -ModuleConstants.kArmSafeStandoffRotation){
            return;
        }
        if(elevatorPosition > ModuleConstants.kElevatorMinimumPoweredHeight){
            if(elevatorPosition - m < 0){
                m_elevatorSparkMax0.set(
                    MathUtil.clamp(
                        m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                        );
                
            }
            if(elevatorPosition - m > 0){
                m_elevatorSparkMax0.set(
                    MathUtil.clamp(
                        m_elevatorPIDControllerDown.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                        );
            }
        }
        else if(elevatorPosition < ModuleConstants.kElevatorMinimumPoweredHeight && desiredElevatorPosition > ModuleConstants.kElevatorMinimumPoweredHeight){
            if(elevatorPosition - m < 0){
                m_elevatorSparkMax0.set(
                    MathUtil.clamp(
                        m_elevatorPIDController.calculate(elevatorPosition, m), 0, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                        );
                
            }
            if(elevatorPosition - m > 0){
                m_elevatorSparkMax0.set(
                    MathUtil.clamp(
                        m_elevatorPIDControllerDown.calculate(elevatorPosition, m), 0, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                        );
            }
        }
        else{
            if(elevatorPosition - m < 0){
                m_elevatorSparkMax0.set(
                    MathUtil.clamp(
                        m_elevatorPIDController.calculate(elevatorPosition, m), 0, 0)*ModuleConstants.kElevatorMotorSpeedMultiplier
                        );
                
            }
            if(elevatorPosition - m > 0){
                m_elevatorSparkMax0.set(
                    MathUtil.clamp(
                        m_elevatorPIDControllerDown.calculate(elevatorPosition, m), 0, 0)*ModuleConstants.kElevatorMotorSpeedMultiplier
                        );
            }
        }
    }


    // In meters
    public void setDesiredHeight(double m){
        desiredElevatorPosition = m;
    }

    public double getElevatorPosition(){
        //if(4 * m_elevatorSparkMax0.getEncoder().getPosition() < m_elevatorAbsoluteEncoder.getPosition())
        //m_elevatorSparkMax0.getEncoder().setPosition(m_elevatorAbsoluteEncoder.getPosition() * 4);

        return Rotation2d.fromRadians(m_elevatorSparkMax0.getEncoder().getPosition()).getRotations() * ModuleConstants.kElevatorSprocketCircumference / ModuleConstants.kElevatorMotorReduction;
    }

    // returns second stage elevator height (from the base) in meters, calculated only by the absolute encoder
    public double getElevatorPositionAbsolute(){
        encoderPosition = Rotation2d.fromRadians(m_elevatorAbsoluteEncoder.getPosition()).getDegrees();
        if(encoderPosition-lastEncoderPosition < -ModuleConstants.kEncoderResetThreshold) encoderRotations += 1;
        else if(encoderPosition-lastEncoderPosition > ModuleConstants.kEncoderResetThreshold) encoderRotations -= 1;
        lastEncoderPosition = encoderPosition;

        return ((encoderPosition)/360+(encoderRotations)) * ModuleConstants.kElevatorSprocketCircumference * 2;
    }

    public void elevateToReef(int reef){
        MathUtil.clamp(reef, 0, AutoConstants.kReefHeights.length-1);

        setDesiredHeight(AutoConstants.kReefHeights[reef]);
    }

    public int getElevatorRegion(double elevatorPos){
        int i = 0;
        for (double pos : ModuleConstants.kArmBlockoffRegions) {
            if(elevatorPos < pos) return i;
            i++;
        }
        return -1;
    }

    public void initiateDashboard(){
        SmartDashboard.putNumber("Elevator Position", elevatorPosition);
        SmartDashboard.putNumber("Desired Elevator Position", desiredElevatorPosition);
    }
    public void updateDashboardValues(){
        SmartDashboard.putNumber("Elevator Position", elevatorPosition);
        SmartDashboard.putNumber("Desired Elevator Position", desiredElevatorPosition);
    }
}

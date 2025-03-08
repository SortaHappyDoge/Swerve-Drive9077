package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    public boolean  armBlocked = true;
    public boolean doMove = false;

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
        /*kElevatorSparkMaxConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.kElevatorPID[0], ModuleConstants.kElevatorPID[1], ModuleConstants.kElevatorPID[2])
            .outputRange(-1, 1);*/

        m_elevatorSparkMax0.configure(kElevatorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kElevatorSparkMaxConfig.follow(m_elevatorSparkMax0, true);
        m_elevatorSparkMax1.configure(kElevatorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //m_elevatorPIDController = m_elevatorSparkMax0.getClosedLoopController();
        m_elevatorAbsoluteEncoder = m_elevatorSparkMax0.getAbsoluteEncoder();
        lastEncoderPosition = m_elevatorAbsoluteEncoder.getPosition();
        encoderRotations = 0;

        m_elevatorPIDController.setPID(ModuleConstants.kElevatorPIDUp[0], ModuleConstants.kElevatorPIDUp[1], ModuleConstants.kElevatorPIDUp[2]);
        m_elevatorPIDController.setTolerance(ModuleConstants.kElevatorPIDTolerance);
        
        m_elevatorPIDControllerDown.setPID(ModuleConstants.kElevatorPIDDown[0], ModuleConstants.kElevatorPIDDown[1], ModuleConstants.kElevatorPIDDown[2]);
        m_elevatorPIDController.setTolerance(ModuleConstants.kElevatorPIDTolerance);
        //initiateDashboard();
    }


    @Override
    public void periodic(){
        elevatorPosition = getElevatorPosition();
        armBlocked = !m_armSubsystem.armStandoffCheck();
        if (armBlocked && manualControl) doMove = false;
        else doMove = true;
        //desiredElevatorPosition = 1.34;
        moveElevatorTo(desiredElevatorPosition);
        //System.out.println(elevatorPosition);
        //updateDashboardValues();
    }

    // In meters
    public void moveElevator(double m){
        moveElevatorTo(elevatorPosition + m);
    }
    public void moveElevatorTo(double m){
        //m_armSubsystem.doRotate = false;
        m = MathUtil.clamp(m, 0, ModuleConstants.kElevatorHeightLimit);
        //m_elevatorSparkMax0.setVoltage(m);
        /*if(!doMove) {
            m_elevatorSparkMax0.stopMotor();
            return;
        }
        else{
            int elevatorRegion = getElevatorRegion(elevatorPosition);
            int desiredElevatorRegion = getElevatorRegion(desiredElevatorPosition);
            m_armSubsystem.doRotate = true;
            switch (elevatorRegion) {
                case 0:
                    if(desiredElevatorRegion == elevatorRegion){
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[0], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                    }
                    else{
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[1], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        if(m_armSubsystem.armStandoffCheck()){
                            m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                        }
                        else
                        m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, ModuleConstants.kArmBlockoffRegions[1]), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                    }
                    break;
                case 1:
                    m_armSubsystem.armMinMaxRotation = new double[]{
                        ModuleConstants.kArmMinBlockedMaxRotations[1], 
                        ModuleConstants.kArmMinBlockedMaxRotations[2]
                    };
                    m_elevatorSparkMax0.set(
                        MathUtil.clamp(
                            m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                            );
                    break;
                case 2:
                    if(desiredElevatorRegion == elevatorRegion){
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[0], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                    }
                    else if(desiredElevatorRegion < elevatorRegion){
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[1], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        if(m_armSubsystem.armStandoffCheck()){
                            m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                        }
                        else{
                            m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, ModuleConstants.kArmBlockoffRegions[2]), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                        }
                    }
                    else{
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[1], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        if(m_armSubsystem.armStandoffCheck()){
                            m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                        }
                        else{
                            m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, ModuleConstants.kArmBlockoffRegions[3]), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                        }
                    }
                    break;
                case 3:
                    m_armSubsystem.armMinMaxRotation = new double[]{
                        ModuleConstants.kArmMinBlockedMaxRotations[1], 
                        ModuleConstants.kArmMinBlockedMaxRotations[2]
                    };
                    m_elevatorSparkMax0.set(
                        MathUtil.clamp(
                            m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                            );
                    break;
                case 4:
                    if(desiredElevatorRegion == elevatorRegion){
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[0], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, m), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                    }
                    else {
                        m_armSubsystem.armMinMaxRotation = new double[]{
                            ModuleConstants.kArmMinBlockedMaxRotations[0], 
                            ModuleConstants.kArmMinBlockedMaxRotations[2]
                        };
                        m_elevatorSparkMax0.set(
                            MathUtil.clamp(
                                m_elevatorPIDController.calculate(elevatorPosition, ModuleConstants.kArmBlockoffRegions[4]), -1, 1)*ModuleConstants.kElevatorMotorSpeedMultiplier
                                );
                    }
                default:
                    break;
            }
        }*/
        if(desiredElevatorPosition > 0.1 || elevatorPosition > 0.1){
            m_armSubsystem.armMinMaxRotation = new double[]{ModuleConstants.kArmMinBlockedMaxRotations[1], ModuleConstants.kArmMinBlockedMaxRotations[2]};
        }
        else{
            m_armSubsystem.armMinMaxRotation = new double[]{ModuleConstants.kArmMinBlockedMaxRotations[0], ModuleConstants.kArmMinBlockedMaxRotations[2]};
        }
        if(Rotation2d.fromRadians(m_armSubsystem.m_armAbsoluteEncoder.getPosition()).getDegrees() < ModuleConstants.kArmMinBlockedMaxRotations[1] -ModuleConstants.kArmSafeStandoffRotation){
            return;
        }
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


    // In meters
    public void setDesiredHeight(double m){
        desiredElevatorPosition = m;
    }

    // returns second stage elevator height (from the base) in meters
    public double getElevatorPosition(){
        encoderPosition = Rotation2d.fromRadians(m_elevatorAbsoluteEncoder.getPosition()).getDegrees();
        if(encoderPosition-lastEncoderPosition < -ModuleConstants.kEncoderResetThreshold) encoderRotations += 1;
        else if(encoderPosition-lastEncoderPosition > ModuleConstants.kEncoderResetThreshold) encoderRotations -= 1;
        lastEncoderPosition = encoderPosition;

        return ((encoderPosition)/360+(encoderRotations)) * ModuleConstants.kElevatorSprocketCircumference * 2;
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
        /*SmartDashboard.putData("Elevator PID", m_elevatorPIDController);
        SmartDashboard.putNumber("Elevator Height", elevatorPosition);
        SmartDashboard.putNumber("Desired Elevator Height", desiredElevatorPosition);*/
    }
    public void updateDashboardValues(){
        //SmartDashboard.updateValues();
    }
}

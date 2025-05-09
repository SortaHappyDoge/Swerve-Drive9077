package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Queue;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutonomousCommands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase{
    
    AutonomousCommands m_autonCmds;
    ElevatorSubsystem m_elevator;

    //QTRModule coralQTR = new QTRModule(ModuleConstants.coralQTRPins, ModuleConstants.coralQTREmitter);
    boolean coralLoaded = false;
    boolean doLoadCoral = false;
    boolean doUnloadCoral = false;
    AnalogInput coralChecker = new AnalogInput(3);

    // Coral position is 1 for when the qtr sensor DOESN'T see the coral and the coral is closer to the HOPPPER
    // Coral position is 0 for when the qtr sensor DOES see the coral
    // Coral position is -1 for when the qtr sensor DOESN'T see the coral and the coral is being dropped
    int coralPosition = 1;

    SparkMax m_armSparkMax;
    SparkMaxConfig kArmSparkMaxConfig;
    SparkAbsoluteEncoder m_armAbsoluteEncoder;
    PIDController m_armPIDController;

    SparkMax m_rollerSparkMax;
    SparkMaxConfig kRollerSparkMaxConfig;
    SparkClosedLoopController m_rollerPIDcontroller;
    
    public boolean doRotate = false;
    public double[] armMinMaxRotation = {ModuleConstants.kArmMinBlockedMaxRotations[0], ModuleConstants.kArmMinBlockedMaxRotations[2]};
    public double desiredArmAngle = 0;
    public double armAngle;

    PIDController dashboard_armPID = new PIDController(ModuleConstants.kArmPID[0], ModuleConstants.kArmPID[1], ModuleConstants.kArmPID[2]);
    PIDController dashboard_rollerPID = new PIDController(ModuleConstants.kRollerPID[0], ModuleConstants.kRollerPID[1], ModuleConstants.kRollerPID[2]);
    
    double coralHoldTimer = 0;

    public ArmSubsystem(AutonomousCommands autonomousCommands, ElevatorSubsystem elevatorSubsystem){
        this.m_autonCmds = autonomousCommands; this.m_elevator = elevatorSubsystem;

        m_armSparkMax = new SparkMax(ModuleConstants.kArmSparkMaxCanID, MotorType.kBrushless);
        kArmSparkMaxConfig = new SparkMaxConfig();
        kArmSparkMaxConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        kArmSparkMaxConfig.absoluteEncoder
            .positionConversionFactor(ModuleConstants.kArmEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kArmEncoderVelocityFactor);
        /*kArmSparkMaxConfig.softLimit
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(ModuleConstants.kArmMinBlockedMaxRotations[0])
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ModuleConstants.kArmMinBlockedMaxRotations[2]);*/
        kArmSparkMaxConfig.openLoopRampRate(ModuleConstants.kArmMotorSpeedRampRate);

        m_armSparkMax.configure(kArmSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_armAbsoluteEncoder = m_armSparkMax.getAbsoluteEncoder();
        m_armPIDController = new PIDController(ModuleConstants.kArmPID[0], ModuleConstants.kArmPID[1], ModuleConstants.kArmPID[2]);

        m_rollerSparkMax = new SparkMax(ModuleConstants.kRollerSparkMaxCanID, MotorType.kBrushless);
        kRollerSparkMaxConfig = new SparkMaxConfig();
        kRollerSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        kRollerSparkMaxConfig.encoder
            .positionConversionFactor(ModuleConstants.kRollerEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kRollerEncoderVelocityFactor);
        kRollerSparkMaxConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.kRollerPID[0], ModuleConstants.kRollerPID[1], ModuleConstants.kRollerPID[2])
            .outputRange(-1, 1)
            .velocityFF(ModuleConstants.kRollerPID[3]);

        m_rollerSparkMax.configure(kRollerSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_rollerPIDcontroller = m_rollerSparkMax.getClosedLoopController();


        initiateDashboard();
    }

    @Override
    public void periodic(){
        if(coralChecker.getVoltage() > 4){
            coralLoaded = false;
            coralHoldTimer = Timer.getFPGATimestamp();
        } 
        else if(Timer.getFPGATimestamp() - coralHoldTimer > 0.05) {
            coralLoaded = true;
        }
        doRotate = true;
        if(doLoadCoral) doUnloadCoral = false;

        armAngle = getArmAngle();
        if(desiredArmAngle >= ModuleConstants.kArmMinimumPoweredRotation || Rotation2d.fromRadians(m_armAbsoluteEncoder.getPosition()).getDegrees() >= ModuleConstants.kArmMinimumPoweredRotation)
        /*if(doRotate) */rotateArmTo(Rotation2d.fromDegrees(desiredArmAngle));
        
        updateDashboardValues();
    }

    public void rotateArmTo(Rotation2d rotation){
        rotation = Rotation2d.fromDegrees(MathUtil.clamp(rotation.getDegrees(), armMinMaxRotation[0], armMinMaxRotation[1]));
        if(rotation.getDegrees() < ModuleConstants.kArmMinimumPoweredRotation && getArmAngle() < ModuleConstants.kArmMinimumPoweredRotation){
            m_armSparkMax.set(MathUtil.clamp(m_armPIDController.calculate(0, 0), -ModuleConstants.kArmMotorSpeedMultiplier, ModuleConstants.kArmMotorSpeedMultiplier));
        }
        else
        m_armSparkMax.set(MathUtil.clamp(m_armPIDController.calculate(armAngle-rotation.getDegrees(), 0), -ModuleConstants.kArmMotorSpeedMultiplier, ModuleConstants.kArmMotorSpeedMultiplier));
    }

    public double getArmAngle(){
        double rotation = Rotation2d.fromRadians(m_armAbsoluteEncoder.getPosition()).getDegrees();
        if(rotation > armMinMaxRotation[1]+20) rotation = 0;

        return rotation;
    }

    public void loadCoralManual(double sped){
        //m_rollerPIDcontroller.setReference(ModuleConstants.kMaxRollerSpeedRPM*0.01, SparkMax.ControlType.kVelocity);
        m_rollerSparkMax.set(sped);
    }
    public FunctionalCommand loadCoral(double sped){
        return new FunctionalCommand(() -> {loadCoralManual(sped);}, 
        () -> {
            if(coralChecker.getVoltage() > 4) loadCoralManual(sped);
            else loadCoralManual(0);
        }, 
        interrupted -> {if(coralLoaded) {loadCoralManual(0);}}, 
        () -> {return coralLoaded;}, 
        this);
    }
    public void unloadCoral(){
        if(coralLoaded){
            m_rollerPIDcontroller.setReference(ModuleConstants.kMaxRollerSpeedRPM * ModuleConstants.coralQTRErrorMultipliers[0], SparkMax.ControlType.kVelocity);
        }
    }


    public boolean armStandoffCheck(){
        if(new Rotation2d(m_armAbsoluteEncoder.getPosition()).getDegrees() > ModuleConstants.kArmMinBlockedMaxRotations[1] &&
           new Rotation2d(m_armAbsoluteEncoder.getPosition()).getDegrees() < ModuleConstants.kArmMinBlockedMaxRotations[2])
           return true;
        else return false;
    }

    // In degrees
    public void setDesiredArmRotation(double degrees){
        desiredArmAngle = degrees;
    }

    public void initiateDashboard(){
        SmartDashboard.putNumber("Arm Rotation", armAngle);
        SmartDashboard.putNumber("Desired Arm Rotation", desiredArmAngle);
        SmartDashboard.putBoolean("is Coral Loaded", coralLoaded);
    }
    public void updateDashboardValues(){
        SmartDashboard.putNumber("Arm Rotation", armAngle);
        SmartDashboard.putNumber("Desired Arm Rotation", desiredArmAngle);
        SmartDashboard.putBoolean("is Coral Loaded", coralLoaded);
    }
}

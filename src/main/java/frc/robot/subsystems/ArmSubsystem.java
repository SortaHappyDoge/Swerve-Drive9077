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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutonomousCommands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase{
    
    AutonomousCommands m_autonCmds;
    ElevatorSubsystem m_elevator;

    QTRModule coralQTR = new QTRModule(ModuleConstants.coralQTRPins, ModuleConstants.coralQTREmitter);
    boolean coralLoaded = false;
    boolean doLoadCoral = false;
    boolean doUnloadCoral = false;

    // Coral position is 1 for when the qtr sensor DOESN'T see the coral and the coral is closer to the HOPPPER
    // Coral position is 0 for when the qtr sensor DOES see the coral
    // Coral position is -1 for when the qtr sensor DOESN'T see the coral and the coral is being dropped
    int coralPosition = 1;

    SparkMax m_armSparkMax;
    SparkMaxConfig kArmSparkMaxConfig;
    SparkAbsoluteEncoder m_armAbsoluteEncoder;
    SparkClosedLoopController m_armPIDController;

    SparkMax m_rollerSparkMax;
    SparkMaxConfig kRollerSparkMaxConfig;
    SparkClosedLoopController m_rollerPIDcontroller;
    
    public boolean doRotate = false;
    public double[] armMinMaxRotation = {ModuleConstants.kArmMinBlockedMaxRotations[0], ModuleConstants.kArmMinBlockedMaxRotations[2]};
    public double desiredArmAngle = 50;


    PIDController dashboard_armPID = new PIDController(ModuleConstants.kArmPID[0], ModuleConstants.kArmPID[1], ModuleConstants.kArmPID[2]);
    PIDController dashboard_rollerPID = new PIDController(ModuleConstants.kRollerPID[0], ModuleConstants.kRollerPID[1], ModuleConstants.kRollerPID[2]);
    
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
        kArmSparkMaxConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.kArmPID[0], ModuleConstants.kArmPID[1], ModuleConstants.kArmPID[2])
            .outputRange(-.44, .44);
        kArmSparkMaxConfig.softLimit
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(ModuleConstants.kArmMinBlockedMaxRotations[0])
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ModuleConstants.kArmMinBlockedMaxRotations[2]);

        m_armSparkMax.configure(kArmSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_armAbsoluteEncoder = m_armSparkMax.getAbsoluteEncoder();
        m_armPIDController = m_armSparkMax.getClosedLoopController();

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
        coralLoaded = coralQTR.getState()[2];
        doRotate = true;
        if(doLoadCoral) doUnloadCoral = false;
        if(doRotate) rotateArmTo(Rotation2d.fromDegrees(desiredArmAngle));
        System.out.println(Rotation2d.fromRadians(m_armAbsoluteEncoder.getPosition()).getDegrees());
    }

    /*public void unloadCoralM() {
        m_rollerPIDcontroller.setReference(ModuleConstants.kMaxRollerSpeedRPM * ModuleConstants.coralQTRErrorMultipliers[0] * 0.2, SparkMax.ControlType.kVelocity);
    }*/

    /*public void terminateLoadM() {
        m_rollerPIDcontroller.setReference(0, SparkMax.ControlType.kVelocity);
    }*/

    public void armTest(double count) {
        rotateArmTo(Rotation2d.fromDegrees(desiredArmAngle));
        desiredArmAngle += count;
    }

    public void rotateArmTo(Rotation2d rotation){
        rotation = Rotation2d.fromDegrees(MathUtil.clamp(rotation.getDegrees(), armMinMaxRotation[0], armMinMaxRotation[1]));
        m_armPIDController.setReference(rotation.getRadians(), SparkMax.ControlType.kPosition);
    }

    public void loadCoralManual(){
        m_rollerPIDcontroller.setReference(ModuleConstants.kMaxRollerSpeedRPM*0.01, SparkMax.ControlType.kVelocity);
    }
    /*public void loadCoralSimple(){
        m_rollerPIDcontroller.setReference(, SparkMax.ControlType.kVelocity);
    }*/
    public void loadCoral(){
        m_rollerPIDcontroller.setReference(ModuleConstants  .kMaxRollerSpeedRPM * m_autonCmds.coralAdjustments(coralQTR.getState()), SparkMax.ControlType.kVelocity);
    }
    public void unloadCoral(){
        if(coralLoaded){
            m_rollerPIDcontroller.setReference(ModuleConstants.kMaxRollerSpeedRPM * ModuleConstants.coralQTRErrorMultipliers[0], SparkMax.ControlType.kVelocity);
        }
    }


    public boolean armStandoffCheck(){
        if(new Rotation2d(m_armAbsoluteEncoder.getPosition()).getDegrees() > ModuleConstants.kArmMinBlockedMaxRotations[1] &&
           new Rotation2d(m_armAbsoluteEncoder.getPosition()).getDegrees() < 360)
           return true;
        else return false;
    }

    // In degrees
    public void setDesiredArmRotation(double degrees){
        desiredArmAngle = degrees;
    }

    public void initiateDashboard(){
        /*
        addChild("QTR: 0", coralQTR.sensorArray[0]);
        addChild("QTR: 1", coralQTR.sensorArray[1]);
        addChild("QTR: 2", coralQTR.sensorArray[2]);
        addChild("QTR: 3", coralQTR.sensorArray[3]);

        SmartDashboard.putData("Arm PID", dashboard_armPID);
        SmartDashboard.putData("Roller PID", dashboard_rollerPID);
        SmartDashboard.putNumber("Desired Arm Rotation", desiredArmAngle);
        */
    }
    public void updateDashboardValues(){
        /*
        kArmSparkMaxConfig.closedLoop
        .pid(dashboard_armPID.getP(), dashboard_armPID.getI(), dashboard_armPID.getD());
        m_armSparkMax.configure(kArmSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kRollerSparkMaxConfig.closedLoop
        .pid(dashboard_rollerPID.getP(), dashboard_rollerPID.getI(), dashboard_rollerPID.getD());
        m_rollerSparkMax.configure(kRollerSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        */
    }
}

package frc.robot;

import org.opencv.core.RotatedRect;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommands {
    private final DriveSubsystem m_robotDrive;
    
    PIDController rotationPIDController = new PIDController(0, 0, 0);
    PIDController coralAdjustmentPIDController = new PIDController(0, 0, 0);

    public PathPlannerPath[] paths = new PathPlannerPath[28];

    public AutonomousCommands(DriveSubsystem driveSubsystem){
        this.m_robotDrive = driveSubsystem;
        rotationPIDController.setPID(AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);
        rotationPIDController.enableContinuousInput(-180, 180);

        coralAdjustmentPIDController.setPID(ModuleConstants.qtrPID[0], ModuleConstants.qtrPID[1], ModuleConstants.qtrPID[2]);

        //paths[0] = PathPlannerPath.fromPathFile("ID 12 Source");
    }

    public void ResetRotationPID(){
        rotationPIDController.reset();
    }
    public void RotateToAngle(double targetAngle, XboxController controller, double kP, double kI, double kD){
        
        if(targetAngle != 0){ 
            m_robotDrive.drive(
            0-MathUtil.applyDeadband(controller.getLeftY()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            0-MathUtil.applyDeadband(controller.getLeftX()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            +MathUtil.clamp(rotationPIDController.calculate(targetAngle-m_robotDrive.getHeading(), 0), -1, 1), 
            true);
        }
        else{
            m_robotDrive.drive(
            0-MathUtil.applyDeadband(controller.getLeftY()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            0-MathUtil.applyDeadband(controller.getLeftX()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((controller.getRawAxis(2))*0.7, OIConstants.kDriveDeadband*3), 
            true);
        }
    }


    // Returns roller speed based on PID using QTR values
    public double coralAdjustments(boolean[] qtrStates){
        if(checkAllFalse(qtrStates)) return ModuleConstants.coralQTRErrorMultipliers[0];
        else if(qtrStates[3]) return ModuleConstants.coralQTRErrorMultipliers[4];
        else if(qtrStates[2]) return ModuleConstants.coralQTRErrorMultipliers[3];
        else if(qtrStates[1]) return ModuleConstants.coralQTRErrorMultipliers[2];
        else return ModuleConstants.coralQTRErrorMultipliers[1];
    }

    public boolean checkAllFalse(boolean[] array){
        for(boolean b : array) if(b) return false;
        return true;
    }


    public void initiateDashboard(){
        SmartDashboard.putData("Rotate PID", rotationPIDController);
        SmartDashboard.putData("Roller PID", coralAdjustmentPIDController);
    }
    public void updateDashboardValues(){
        
    }
}

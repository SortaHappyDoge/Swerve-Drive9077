package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommands {
    private final DriveSubsystem m_robotDrive;
    public double errorDegrees;
    public double errorSum = 0;
    /*public final double kP = 0.009;
    public final double kI = 0;
    public final double kD = 0.0035;*/
    public double integralLimit = 2;
    public double integralResetPoint = 0.1;
    public double lastTimestamp = Timer.getFPGATimestamp();
    public double lastError = 0;
    public double errorRate = 0;
    public AutonomousCommands(DriveSubsystem driveSubsystem){
        this.m_robotDrive = driveSubsystem;
        //SendableRegistry.setName(, "kP");
    }

    public void ResetPID(){
        errorSum = 0;
        lastError = 0;
    }
    public void RotateToAngle(double targetAngle, XboxController controller, double kP, double kI, double kD){
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        errorDegrees = targetAngle/*targetAngle - ((m_robotDrive.getHeading()+180) % 360 + 360) % 360 -180*/;
        if(Math.abs(errorDegrees) < integralLimit){ errorSum += dt*errorDegrees; }
        else{ errorSum = 0; }
        if(Math.abs(errorDegrees) < integralResetPoint) { errorSum = 0; }
        errorRate = (errorDegrees - lastError) / dt;
        double output = kP * errorDegrees + kI * errorSum + kD * errorRate;
        if(targetAngle != 0){ 
            m_robotDrive.drive(
            0-MathUtil.applyDeadband(controller.getLeftY()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            0-MathUtil.applyDeadband(controller.getLeftX()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            +MathUtil.clamp(output, -1, 1), 
            true);
        }
        else{
            m_robotDrive.drive(
            0-MathUtil.applyDeadband(controller.getLeftY()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            0-MathUtil.applyDeadband(controller.getLeftX()*((controller.getRawAxis(3)*(-1)+1)/2), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((controller.getRawAxis(2))*0.7, OIConstants.kDriveDeadband*3), 
            true);
        }
        lastError = errorDegrees;
        lastTimestamp = Timer.getFPGATimestamp();
    }
}

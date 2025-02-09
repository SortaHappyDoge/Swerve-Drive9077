package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommands {
    private final DriveSubsystem m_robotDrive;
    public double errorDegrees;
    public double errorSum = 0;
    public final double kP = 0.01;
    public final double kI = 0.18;
    public final double kD = 0.001;
    public double integralLimit = 2;
    public double integralResetPoint = 0.02;
    public double lastTimestamp = Timer.getFPGATimestamp();
    public double lastError = 0;
    public double errorRate = 0;
    public AutonomousCommands(DriveSubsystem driveSubsystem){
        this.m_robotDrive = driveSubsystem;
    }

    public void ResetPID(){
        errorSum = 0;
        lastError = 0;
    }
    public void RotateToAngle(double targetAngle){
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        errorDegrees = targetAngle/*targetAngle - ((m_robotDrive.getHeading()+180) % 360 + 360) % 360 -180*/;
        if(Math.abs(errorDegrees) < integralLimit){ errorSum += dt*errorDegrees; }
        if(Math.abs(errorDegrees) < integralResetPoint) { errorSum = 0; }
        errorRate = (errorDegrees - lastError) / dt;
        double output = kP * errorDegrees + kI * errorSum + kD * errorRate;
        m_robotDrive.drive(0, 0, MathUtil.clamp(output, -1, 1), true);
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = errorDegrees;
    }
}

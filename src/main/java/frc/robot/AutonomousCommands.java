package frc.robot;

import java.nio.file.Path;

import org.opencv.core.RotatedRect;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommands {
    private final DriveSubsystem m_robotDrive;
    
    PIDController rotationPIDController = new PIDController(0, 0, 0);
    PIDController coralAdjustmentPIDController = new PIDController(0, 0, 0);

    public PathPlannerPath[] paths = new PathPlannerPath[28];
    public String[] pathLabels = {
        "ID 0",
        "ID 1 Source",
        "ID 2 Source",
        "ID 3",
        "ID 4",
        "ID 5",
        "ID 6 Score L",
        "ID 6 Score R",
        "ID 7 Score L",
        "ID 7 Score R",
        "ID 8 Score L",
        "ID 8 Score R",
        "ID 9 Score L",
        "ID 9 Score R",
        "ID 10 Score L",
        "ID 10 Score R",
        "ID 11 Score L",
        "ID 11 Score R",

        "ID 12 Source",
        "ID 13 Source",
        "ID 14",
        "ID 15",
        "ID 16",
        "ID 17 Score L",
        "ID 17 Score R",
        "ID 18 Score L",
        "ID 18 Score R",
        "ID 19 Score L",
        "ID 19 Score R",
        "ID 20 Score L",
        "ID 20 Score R",
        "ID 21 Score L",
        "ID 21 Score R",
        "ID 22 Score L",
        "ID 22 Score R",
    };
    public Pose2d[] autonDestinations = {
        new Pose2d(3.740,3.036, Rotation2d.fromDegrees(60)),  // Reef ID 17 L
        new Pose2d(4.010,2.884, Rotation2d.fromDegrees(60)),  // Reef ID 17 R
        new Pose2d(1.367,0.758, Rotation2d.fromDegrees(54)),    // Source ID 12
    };

    public int autonState = 0;
    int[] autonSequence = {6, 12, 17, 12, 17};
    double[] elevatorLevelSequence = {
        AutoConstants.kReefHeights[1],
        0,
        AutoConstants.kReefHeights[1],
        0,
        AutoConstants.kReefHeights[1],
        0
    };
    boolean raiseElevator;
    boolean loadCoral;
    boolean unloadCoral;
    boolean readyForInput; 
    boolean readyToPounce;

    public AutonomousCommands(DriveSubsystem driveSubsystem){
        this.m_robotDrive = driveSubsystem;
        rotationPIDController.setPID(AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);
        rotationPIDController.enableContinuousInput(-180, 180);

        coralAdjustmentPIDController.setPID(ModuleConstants.qtrPID[0], ModuleConstants.qtrPID[1], ModuleConstants.qtrPID[2]);

        //setupPaths();

        initiateDashboard();
        updateDashboardValues();
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
        /*SmartDashboard.putData("Rotate PID", rotationPIDController);
        SmartDashboard.putData("Roller PID", coralAdjustmentPIDController);*/
        SmartDashboard.putNumber("Auton State", autonState);
    }
    public void updateDashboardValues(){
        SmartDashboard.updateValues();
    }

    public Command pathfindToPath(int id){
        try {
            PathPlannerPath path = paths[id];
            return AutoBuilder.pathfindThenFollowPath(path,
                AutoConstants.kPathfindingConstraints);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return null;
    }

    public void setupPaths(){
        int i = 0;
        for (PathPlannerPath n : paths) {
            try {
                paths[i] = PathPlannerPath.fromPathFile(pathLabels[i]);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    public void registerAutonCommands(){
        NamedCommands.registerCommand("Ready for Input", new RunCommand(() -> isReadyForInput()));
        NamedCommands.registerCommand("Load Coral", new RunCommand(() -> doLoadCoral()));
        NamedCommands.registerCommand("Raise Elevator", new RunCommand(() -> doRaiseElevator()));
        NamedCommands.registerCommand("Unload Coral", new RunCommand(() -> doUnloadCoral()));
    }
    public void isReadyForInput(){readyForInput = true;}
    public void doLoadCoral(){loadCoral = true;}
    public void doUnloadCoral(){unloadCoral = true;}
    public void doRaiseElevator(){raiseElevator = true;}
    public void isReadyToPounce(){readyToPounce = true;}

    public void autonSequence(){
        if(autonState == 0) return;

        System.out.println(autonState);

        if(autonState == 1){
            autonState = 2;
            Commands.run(() -> pathfindToPath(autonSequence[0]));
            return;
        }
        if(autonState == 2){
            if(raiseElevator){autonState = 3; raiseElevator = false;}
            return;
        }
        if(autonState == 3){
            m_robotDrive.m_elevator.setDesiredHeight(elevatorLevelSequence[0]);
            autonState = 4;
            return;
        }
        if(autonState == 4){
            if(unloadCoral){autonState = 5;} // Unload coral must be set to false from some form of coral detection code
            return;
        }
        if(autonState == 5){
            m_robotDrive.m_armSubsystem.loadCoralManual(0.2);
            if(!unloadCoral) {
                m_robotDrive.m_armSubsystem.loadCoralManual(0);
                autonState = 6;
            }
            return;
        }
        if(autonState == 6){
            autonState = 7;
            Commands.runOnce(() -> pathfindToPath(autonSequence[1]));
            return;
        }
        if(autonState == 7){
            if(readyForInput){autonState = 8; readyForInput = false;}
            return;
        }
        if(autonState == 8){
            if(loadCoral){
                m_robotDrive.m_armSubsystem.loadCoralManual(0.2);
                autonState = 9;
            }
            return;
        }
        if(autonState == 9){
            if(!loadCoral){
                m_robotDrive.m_armSubsystem.loadCoralManual(0);
                autonState = 10;
            }
        }

    }
}

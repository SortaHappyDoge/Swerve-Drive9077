package frc.robot;

import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import javax.lang.model.util.ElementScanner14;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommands {
    public Command currentCommand;
    
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

    
    public PathPlannerPath ID6SCORE_R;
    public PathPlannerPath ID6SCORE_L;
    public PathPlannerPath ID7SCORE_R;
    public PathPlannerPath ID7SCORE_L;
    public PathPlannerPath ID8SCORE_R;
    public PathPlannerPath ID8SCORE_L;
    public PathPlannerPath ID9SCORE_R;
    public PathPlannerPath ID9SCORE_L;
    public PathPlannerPath ID10SCORE_R;
    public PathPlannerPath ID10SCORE_L;
    public PathPlannerPath ID11SCORE_R;
    public PathPlannerPath ID11SCORE_L;
    public PathPlannerPath ID6BALL;
    public PathPlannerPath ID7BALL;
    public PathPlannerPath ID8BALL;
    public PathPlannerPath ID9BALL;
    public PathPlannerPath ID10BALL;
    public PathPlannerPath ID11BALL;



    public PathPlannerPath ID12SOURCE;
    public PathPlannerPath ID13SOURCE;
    public PathPlannerPath ID17SCORE_R;
    public PathPlannerPath ID17SCORE_L;
    public PathPlannerPath ID18SCORE_R;
    public PathPlannerPath ID18SCORE_L;
    public PathPlannerPath ID19SCORE_R;
    public PathPlannerPath ID19SCORE_L;
    public PathPlannerPath ID20SCORE_R;
    public PathPlannerPath ID20SCORE_L;
    public PathPlannerPath ID21SCORE_R;
    public PathPlannerPath ID21SCORE_L;
    public PathPlannerPath ID22SCORE_R;
    public PathPlannerPath ID22SCORE_L;
    public PathPlannerPath ID17BALL;
    public PathPlannerPath ID18BALL;
    public PathPlannerPath ID19BALL;
    public PathPlannerPath ID20BALL;
    public PathPlannerPath ID21BALL;
    public PathPlannerPath ID22BALL;


    public PathPlannerPath AUTO_ID22LSEQUENCE;
    public PathPlannerPath AUTO_ID22to12LSEQUENCE;
    public PathPlannerPath AUTO_ID17RSEQUENCE;
    public PathPlannerPath AUTO_ID17to12LSEQUENCE;
    public PathPlannerPath AUTO_ID17LSEQUENCE;

    public Pose2d[] autonDestinations = {
        new Pose2d(1.367,0.758, Rotation2d.fromDegrees(54)),  // Source ID 12
        new Pose2d(1.366,7.244, Rotation2d.fromDegrees(-54)),  // Source ID 13
        new Pose2d(3.740,3.036, Rotation2d.fromDegrees(60)),  // Reef ID 17 L
        new Pose2d(4.010,2.884, Rotation2d.fromDegrees(60)),  // Reef ID 17 R
        new Pose2d(3.256,4.167, Rotation2d.fromDegrees(0)),  // Reef ID 18 L
        new Pose2d(3.256,3.857, Rotation2d.fromDegrees(0)),  // Reef ID 18 R
        new Pose2d(4.004,5.206, Rotation2d.fromDegrees(-60)),  // Reef ID 19 L
        new Pose2d(3.704,5, Rotation2d.fromDegrees(-60)),  // Reef ID 19 R
        new Pose2d(5.251,5.014, Rotation2d.fromDegrees(-120)),  // Reef ID 20 L
        new Pose2d(4.963,5.170, Rotation2d.fromDegrees(-120)),  // Reef ID 20 R
        new Pose2d(5.742,4.175, Rotation2d.fromDegrees(180)),  // Reef ID 21 L
        new Pose2d(5.742,3.867, Rotation2d.fromDegrees(180)),  // Reef ID 21 R
        new Pose2d(4.998,2.880, Rotation2d.fromDegrees(120)),  // Reef ID 22 L
        new Pose2d(5.251,3.024, Rotation2d.fromDegrees(120)),  // Reef ID 22 R
    };


    double time = 0; double lastTime;

    public AutonomousCommands(DriveSubsystem driveSubsystem){
        this.m_robotDrive = driveSubsystem;
        NamedCommands.registerCommand("Ready Elevator", readyElevator());
        NamedCommands.registerCommand("Raise Elevator 3", raiseElevatorConditionless(3));
        NamedCommands.registerCommand("Lower Elevator", raiseElevatorConditionless(0));
        NamedCommands.registerCommand("Load Coral", loadCoral());
        NamedCommands.registerCommand("Unload Coral", unloadCoralAuton());
        //NamedCommands.registerCommand("Unload Conditioned", unloadCoralConditioned());
        NamedCommands.registerCommand("Ball Level Low", raiseElevatorBalls(1));
        NamedCommands.registerCommand("Ball Level High", raiseElevatorBalls(2));
        NamedCommands.registerCommand("Load Ball", new InstantCommand(() -> {m_robotDrive.m_armSubsystem.loadCoralManual(0.3);}));


        rotationPIDController.setPID(AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD);
        rotationPIDController.enableContinuousInput(-180, 180);
        rotationPIDController.setTolerance(3);

        coralAdjustmentPIDController.setPID(ModuleConstants.qtrPID[0], ModuleConstants.qtrPID[1], ModuleConstants.qtrPID[2]);

        try {
            /*ID6SCORE_R = PathPlannerPath.fromPathFile("ID 6 SCORE R");
            ID6SCORE_L = PathPlannerPath.fromPathFile("ID 6 SCORE L");
            ID7SCORE_R = PathPlannerPath.fromPathFile("ID 7 SCORE R");
            ID7SCORE_L = PathPlannerPath.fromPathFile("ID 7 SCORE L");
            ID8SCORE_R = PathPlannerPath.fromPathFile("ID 8 SCORE R");
            ID8SCORE_L = PathPlannerPath.fromPathFile("ID 8 SCORE L");
            ID9SCORE_R = PathPlannerPath.fromPathFile("ID 9 SCORE R");
            ID9SCORE_L = PathPlannerPath.fromPathFile("ID 9 SCORE L");
            ID10SCORE_R = PathPlannerPath.fromPathFile("ID 10 SCORE R");
            ID10SCORE_L = PathPlannerPath.fromPathFile("ID 10 SCORE L");
            ID11SCORE_R = PathPlannerPath.fromPathFile("ID 11 SCORE R");
            ID11SCORE_L = PathPlannerPath.fromPathFile("ID 11 SCORE L");
            ID6BALL = PathPlannerPath.fromPathFile("ID 6 BALL");
            ID7BALL = PathPlannerPath.fromPathFile("ID 7 BALL");
            ID8BALL = PathPlannerPath.fromPathFile("ID 8 BALL");
            ID9BALL = PathPlannerPath.fromPathFile("ID 9 BALL");
            ID10BALL = PathPlannerPath.fromPathFile("ID 10 BALL");
            ID11BALL = PathPlannerPath.fromPathFile("ID 11 BALL");*/
            
            ID12SOURCE = PathPlannerPath.fromPathFile("ID 12 SOURCE");
            ID13SOURCE = PathPlannerPath.fromPathFile("ID 13 SOURCE");
            ID17SCORE_R = PathPlannerPath.fromPathFile("ID 17 SCORE R");
            ID17SCORE_L = PathPlannerPath.fromPathFile("ID 17 SCORE L");
            ID18SCORE_R = PathPlannerPath.fromPathFile("ID 18 SCORE R");
            ID18SCORE_L = PathPlannerPath.fromPathFile("ID 18 SCORE L");
            ID19SCORE_R = PathPlannerPath.fromPathFile("ID 19 SCORE R");
            ID19SCORE_L = PathPlannerPath.fromPathFile("ID 19 SCORE L");
            ID20SCORE_R = PathPlannerPath.fromPathFile("ID 20 SCORE R");
            ID20SCORE_L = PathPlannerPath.fromPathFile("ID 20 SCORE L");
            ID21SCORE_R = PathPlannerPath.fromPathFile("ID 21 SCORE R");
            ID21SCORE_L = PathPlannerPath.fromPathFile("ID 21 SCORE L");
            ID22SCORE_R = PathPlannerPath.fromPathFile("ID 22 SCORE R");
            ID22SCORE_L = PathPlannerPath.fromPathFile("ID 22 SCORE L");
            ID17BALL = PathPlannerPath.fromPathFile("ID 17 BALL");
            ID18BALL = PathPlannerPath.fromPathFile("ID 18 BALL");
            ID19BALL = PathPlannerPath.fromPathFile("ID 19 BALL");
            ID20BALL = PathPlannerPath.fromPathFile("ID 20 BALL");
            ID21BALL = PathPlannerPath.fromPathFile("ID 21 BALL");
            ID22BALL = PathPlannerPath.fromPathFile("ID 22 BALL");


            AUTO_ID22LSEQUENCE = PathPlannerPath.fromPathFile("Auto SCORE ID 22 L SEQUENCE");
            AUTO_ID22to12LSEQUENCE = PathPlannerPath.fromPathFile("Auto SOURCE 22-12 SEQUENCE");
            AUTO_ID17RSEQUENCE = PathPlannerPath.fromPathFile("Auto SCORE ID 17 R SEQUENCE");
            AUTO_ID17to12LSEQUENCE = PathPlannerPath.fromPathFile("Auto SOURCE 17-12 SEQUENCE");
            AUTO_ID17LSEQUENCE = PathPlannerPath.fromPathFile("Auto SCORE ID 17 L SEQUENCE");
        } catch (Exception e) {
            System.out.print(e);
        }


        initiateDashboard();
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

    
    public Command selectReef(boolean isRight, XboxController controller, int POV){
        boolean isBlue;
        if(DriverStation.getAlliance().get() == Alliance.Red)
        isBlue = false;
        else 
        isBlue = true;
                
        if(isBlue)
        switch (POV) {
            case 45:
                return pathfindToReef(22, isRight, isBlue);
            case 135:
                return pathfindToReef(17, isRight, isBlue);
            case 180:
                return pathfindToReef(18, isRight, isBlue);
            case 225:
                return pathfindToReef(19, isRight, isBlue);
            case 315:
                return pathfindToReef(20, isRight, isBlue);
            case 0:
                return pathfindToReef(21, isRight, isBlue);
            case 90:
                CommandScheduler.getInstance().cancelAll();
                break;
            case 270:
                CommandScheduler.getInstance().cancelAll();
                break;
        }
        else
        switch (POV) {
            case 45:
                return pathfindToReef(6, isRight, isBlue);
            case 135:
                return pathfindToReef(11, isRight, isBlue);
            case 180:
                return pathfindToReef(10, isRight, isBlue);
            case 225:
                return pathfindToReef(9, isRight, isBlue);
            case 315:
                return pathfindToReef(8, isRight, isBlue);
            case 0:
                return pathfindToReef(7, isRight, isBlue);
            case 90:
                CommandScheduler.getInstance().cancelAll();
                break;
            case 270:
                CommandScheduler.getInstance().cancelAll();
                break;
        }
        return null;
    }

    public Command pathfindToReef(int id, boolean isRight, boolean isBlue){
        if(!isRight && isBlue)
        switch (id) {
            case 12:
                return AutoBuilder.pathfindThenFollowPath(ID12SOURCE, AutoConstants.kPathfindingConstraints);
            case 13:
                return AutoBuilder.pathfindThenFollowPath(ID13SOURCE, AutoConstants.kPathfindingConstraints);
            case 17:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_L, AutoConstants.kPathfindingConstraints);
            case 18:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_L, AutoConstants.kPathfindingConstraints);                
            case 19:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_L, AutoConstants.kPathfindingConstraints);
            case 20:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_L, AutoConstants.kPathfindingConstraints);
            case 21:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_L, AutoConstants.kPathfindingConstraints);
            case 22:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_L, AutoConstants.kPathfindingConstraints);   
        }
        else if(isRight && isBlue)
        switch (id) {
            case 12:
                return AutoBuilder.pathfindThenFollowPath(ID12SOURCE, AutoConstants.kPathfindingConstraints);
            case 13:
                return AutoBuilder.pathfindThenFollowPath(ID13SOURCE, AutoConstants.kPathfindingConstraints);
            case 17:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_R, AutoConstants.kPathfindingConstraints);
            case 18:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_R, AutoConstants.kPathfindingConstraints);                
            case 19:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_R, AutoConstants.kPathfindingConstraints);
            case 20:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_R, AutoConstants.kPathfindingConstraints);
            case 21:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_R, AutoConstants.kPathfindingConstraints);
            case 22:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_R, AutoConstants.kPathfindingConstraints);            
        }
        if(!isRight && !isBlue)
        switch (id) {
            case 11:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_L, AutoConstants.kPathfindingConstraints);
            case 10:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_L, AutoConstants.kPathfindingConstraints);                
            case 9:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_L, AutoConstants.kPathfindingConstraints);
            case 8:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_L, AutoConstants.kPathfindingConstraints);
            case 7:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_L, AutoConstants.kPathfindingConstraints);
            case 6:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_L, AutoConstants.kPathfindingConstraints);
        }
        else if(isRight && !isBlue)
        switch (id) {
            case 11:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_R, AutoConstants.kPathfindingConstraints);
            case 10:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_R, AutoConstants.kPathfindingConstraints);                
            case 9:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_R, AutoConstants.kPathfindingConstraints);
            case 8:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_R, AutoConstants.kPathfindingConstraints);
            case 7:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_R, AutoConstants.kPathfindingConstraints);
            case 6:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_R, AutoConstants.kPathfindingConstraints);
        }
        System.out.println("is null for some god forsaken reason ????");
        return null;
    }

    public Command selectBall(XboxController controller, int POV){
        boolean isBlu;
        if(DriverStation.getAlliance().get() == Alliance.Red)
        isBlu = false;
        else 
        isBlu = true;
        System.out.println(POV);

        //if(isBlu)
        switch (POV) {
            case 0:
                return pathfindToBall(21, isBlu);
            case 45:
                return pathfindToBall(22, isBlu);
            case 135:
                return pathfindToBall(17, isBlu);
            case 180:
                return pathfindToBall(18, isBlu);
            case 225:
                return pathfindToBall(19, isBlu);
            case 315:
                return pathfindToBall(20, isBlu);
            case 90:
                CommandScheduler.getInstance().cancelAll();
                break;
            case 270:
                CommandScheduler.getInstance().cancelAll();
                break;
        }
        System.out.println("is null for some god forsaken reason x2 ???");

        return null;
    }
    public Command pathfindToBall(int id, boolean isBlue){
        if(isBlue)
        switch (id) {
            case 17:
                return AutoBuilder.pathfindThenFollowPath(ID17BALL, AutoConstants.kPathfindingConstraints);
            case 18:
                return AutoBuilder.pathfindThenFollowPath(ID18BALL, AutoConstants.kPathfindingConstraints);
            case 19:
                return AutoBuilder.pathfindThenFollowPath(ID19BALL, AutoConstants.kPathfindingConstraints);
            case 20:
                return AutoBuilder.pathfindThenFollowPath(ID20BALL, AutoConstants.kPathfindingConstraints);
            case 21:
                return AutoBuilder.pathfindThenFollowPath(ID21BALL, AutoConstants.kPathfindingConstraints);
            case 22:
                return AutoBuilder.pathfindThenFollowPath(ID22BALL, AutoConstants.kPathfindingConstraints);
        }
        else
        switch (id) {
            case 6:
                return AutoBuilder.pathfindThenFollowPath(ID22BALL, AutoConstants.kPathfindingConstraints);
            case 7:
                return AutoBuilder.pathfindThenFollowPath(ID21BALL, AutoConstants.kPathfindingConstraints);
            case 8:
                return AutoBuilder.pathfindThenFollowPath(ID20BALL, AutoConstants.kPathfindingConstraints);
            case 9:
                return AutoBuilder.pathfindThenFollowPath(ID19BALL, AutoConstants.kPathfindingConstraints);
            case 10:
                return AutoBuilder.pathfindThenFollowPath(ID18BALL, AutoConstants.kPathfindingConstraints);
            case 11:
                return AutoBuilder.pathfindThenFollowPath(ID17BALL, AutoConstants.kPathfindingConstraints);
        }
        System.out.println("is null for some god forsaken reason ???");
        return null;
    }
    public void initiateDashboard(){
        /*SmartDashboard.putData("Rotate PID", rotationPIDController);
        SmartDashboard.putData("Roller PID", coralAdjustmentPIDController);*/
    }
    public void updateDashboardValues(){
        //SmartDashboard.updateValues();
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
    

    public Command loadCoral(){
        return m_robotDrive.m_armSubsystem.loadCoral(-0.3);
    }

    public Command unloadCoralAuton(){
        FunctionalCommand cmd = new FunctionalCommand(
            () -> {unloadCoral().schedule();}, 
            () -> {}, 
            interrupted -> {},
            () -> {return true;},
            m_robotDrive.m_armSubsystem);
        
        return cmd;
    }
    public Command unloadCoral(){
        FunctionalCommand cmd = new FunctionalCommand(
            () -> {m_robotDrive.m_armSubsystem.loadCoralManual(-1); time = Timer.getFPGATimestamp();}, 
            () -> {}, 
            interrupted -> {
                m_robotDrive.m_armSubsystem.loadCoralManual(0);
                m_robotDrive.m_armSubsystem.setDesiredArmRotation(ModuleConstants.kArmMinBlockedMaxRotations[1]);
                m_robotDrive.m_elevator.setDesiredHeight(0);
            },
            () -> {return Timer.getFPGATimestamp() - time > 0.5;},
            m_robotDrive.m_armSubsystem);
        
        return cmd;
    }
    public Command unloadCoralConditioned(){
        FunctionalCommand unloadConditioned = new FunctionalCommand(
            () -> {if(m_robotDrive.m_elevator.atTarget) unloadCoral().schedule();}, 
            () -> {}, 
            interrupted -> {}, 
            () -> {return true;}
            );
        return unloadConditioned;
    }

    public Command raiseElevatorAuton(){
        //if(DriverStation.isTeleop()) return new WaitCommand(0);
        FunctionalCommand elevate = new FunctionalCommand(
            () -> {new InstantCommand(() -> m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kReefHeights[3]))
                .andThen(new InstantCommand(() -> m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[3]))).schedule();
            },
            () -> {}, 
            interrupted -> {},
            () -> {return true; }
            );
        return elevate;
    }
    public Command raiseElevator(int level){
        FunctionalCommand elevate = new FunctionalCommand(
            () -> {
                m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kReefHeights[level]); 
                m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[level]);
            }, 
            () -> {}, 
            interrupted -> {
                //m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[level]);
            },
            () -> {return m_robotDrive.m_elevator.getElevatorPosition() > AutoConstants.kReefHeights[level] - AutoConstants.kHeightTolerance;},
            m_robotDrive.m_elevator);
        return elevate;
    }
    public Command raiseElevatorConditionless(int level){
        FunctionalCommand elevate = new FunctionalCommand(
            () -> {m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kReefHeights[level]); 
                m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[level]);
            }, 
            () -> {}, 
            interrupted -> {},
            () -> {return true;},
            m_robotDrive.m_elevator);
        return elevate;
    }
    public Command readyElevator(){
        FunctionalCommand readyElevator = new FunctionalCommand(
            () -> {raiseElevatorConditionless(4);}, 
            () -> {},
            interrupted -> {},
            () -> {return true;},
            m_robotDrive.m_elevator);
        return readyElevator;
    }
    public Command raiseElevatorBalls(int level){
        FunctionalCommand raise = new FunctionalCommand(
            () -> {
                m_robotDrive.m_elevator.setDesiredHeight(AutoConstants.kBallHeights[level]);
                m_robotDrive.m_armSubsystem.setDesiredArmRotation(AutoConstants.kReefAngles[4]);
            }, 
            () -> {}, 
            interrupted -> {}, 
            () -> {return true;}, 
            m_robotDrive.m_elevator);
        return raise;
    }


    public Command scoreCoral(int level){
        Command cmd = raiseElevator(level);
        Command cmd2 = unloadCoral();
        Command cmd3 = raiseElevator(0);
        Command cmd4 = new InstantCommand(() -> m_robotDrive.m_armSubsystem.setDesiredArmRotation(0));
        return cmd.andThen(cmd2).andThen(cmd3).andThen(cmd4);
    }

    public void setupPathPlannerCommands(){
        
    }
}

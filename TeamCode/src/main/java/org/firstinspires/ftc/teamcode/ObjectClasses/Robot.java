package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class Robot {

    private static Robot robot = null;
    public RobotType robotType;
    public OpModeType opModeType;
    public enum RobotType {ROBOT_INTOTHEDEEP, ROBOT_CHASSIS, ROBOT_PIT_MODE, ROBOT_CENTERSTAGE_PODS, ROBOT_CENTERSTAGE_OTOS}
    public enum OpModeType {TELEOP, AUTO}

    private static LinearOpMode activeOpMode;
    private static DriveSubsystem mecanumDriveSubsystem;
    private static GyroSubsystem gyroSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static GripperSubsystem gripperSubsystem;
    private static LiftSubsystem liftSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static ClimberSubsystem climberSubsystem;

    /* Constructor */
    private Robot(LinearOpMode opMode, RobotType rType) {
        activeOpMode = opMode;
        robotType = rType;
        HardwareMap hMap = opMode.hardwareMap;
        CreateSubsystems(hMap);
    }

    private void CreateSubsystems(HardwareMap hardwareMap) {
        switch (robotType) {
            //Just the drive base
            case ROBOT_CHASSIS: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                gyroSubsystem = new GyroSubsystem();
//                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                break;
            }
            case ROBOT_INTOTHEDEEP: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                gyroSubsystem = new GyroSubsystem();
//                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");

//                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake", "intake2");
//                gripperSubsystem = new GripperSubsystem(hardwareMap, "endeffector");
//                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
//                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
//                climberSubsystem = new ClimberSubsystem(hardwareMap, "climb", "climbWinch");
            }

            case ROBOT_PIT_MODE: {
                break;
            }

            case ROBOT_CENTERSTAGE_OTOS:
            case ROBOT_CENTERSTAGE_PODS: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                gyroSubsystem = new GyroSubsystem();
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam");
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "par", "perp");
                gripperSubsystem = new GripperSubsystem(hardwareMap, "endeffector");
                liftSubsystem = new LiftSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                climberSubsystem = new ClimberSubsystem(hardwareMap, "climb", "climbWinch");
                break;
            }

        }
    }

    //Ensures only one robot object is ever created
    public static void createInstance(LinearOpMode opMode, RobotType robotType) {
        //there should only ever be one instance of robot - when we run a new opMode it should delete any previously existing robot object and make a new one
        //MatchConfig can house any data we need between opModes.
        robot = null;
        robot = new Robot(opMode, robotType);
    }

    public synchronized void reset() {
        robot = null;
    }


    // Static method to get single instance of Robot
    public static synchronized Robot getInstance() {
        if (robot == null) {
            telemetry.addLine("error");
        }
        return robot;
    }


    // Initialize teleop or autonomous, depending on which is used
    public void init(OpModeType oType){
        opModeType = oType;
            if (opModeType == OpModeType.TELEOP) {
                initTele();
            } else {
                initAuto();
            }
        }


    private void initTele() {
        switch (robotType) {
            case ROBOT_CHASSIS: {
                gyroSubsystem.init();
//                visionSubsystem.init();
                mecanumDriveSubsystem.init();
                break;
            }

            case ROBOT_INTOTHEDEEP: {
                gyroSubsystem.init();
//                visionSubsystem.init();
                mecanumDriveSubsystem.init();
//                intakeSubsystem.init();
//                gripperSubsystem.init();
//                liftSlideSubsystem.init();
//                shoulderSubsystem.init();
//                climberSubsystem.init();
                break;
            }
            case ROBOT_PIT_MODE: {
                break;
            }

            case ROBOT_CENTERSTAGE_PODS: {
                visionSubsystem.init();
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                intakeSubsystem.init();
                gripperSubsystem.init();
                liftSubsystem.init();
                shoulderSubsystem.init();
                climberSubsystem.init();
                break;
            }
        }
    }

    private void initAuto() {
        // initialize auto-specific scheduler
        switch (robotType) {
            case ROBOT_INTOTHEDEEP:
//                visionSubsystem.init();
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
//                intakeSubsystem.init();
//                gripperSubsystem.init();
//                liftSlideSubsystem.init();
//                shoulderSubsystem.init();
                break;

            case ROBOT_CENTERSTAGE_PODS: {
                visionSubsystem.init();
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                intakeSubsystem.init();
                gripperSubsystem.init();
                liftSubsystem.init();
                shoulderSubsystem.init();
                break;
            }
        }
    }

    public GyroSubsystem getGyroSubsystem()  {return gyroSubsystem;    }
    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public IntakeSubsystem getIntakeSubsystem()  {return intakeSubsystem;}
    public GripperSubsystem getEndEffectorSubsystem()  {return gripperSubsystem;}
    public LiftSubsystem getLiftSubsystem()  {return liftSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}
    public ClimberSubsystem getClimberSubsystem(){return climberSubsystem;};

}




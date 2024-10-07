package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

import java.util.EnumSet;
import java.util.Set;

public class Robot {

    private static Robot robot = null;

    private DriverStationTelemetryManager driverStationTelemetryManager;

    public RobotType robotType;
    public OpModeType opModeType;
    public enum RobotType {
        CHASSIS_19429_A_PINPOINT,
        CHASSIS_19429_HUB_TWO_DEAD_WHEELS,
        CENTERSTAGE_OTOS,
        CENTERSTAGE_HUB_TWO_DEAD_WHEELS
    }
    public enum OpModeType {TELEOP, AUTO}

    private static LinearOpMode activeOpMode;
    private static DriveSubsystem mecanumDriveSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static GripperSubsystem gripperSubsystem;
    private static LiftSubsystem liftSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static ClimberSubsystem climberSubsystem;

    public enum SubsystemType {
        DRIVE, GYRO, VISION, INTAKE, GRIPPER, LIFT, SHOULDER, CLIMBER
    }

    // Use an EnumSet for tracking available subsystems
    private final Set<SubsystemType> availableSubsystems = EnumSet.noneOf(SubsystemType.class);

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
            case CHASSIS_19429_HUB_TWO_DEAD_WHEELS:
            case CHASSIS_19429_A_PINPOINT: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, robotType);
                break;
            }

            case CENTERSTAGE_OTOS:
            case CENTERSTAGE_HUB_TWO_DEAD_WHEELS:{
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, robotType);
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
            activeOpMode.telemetry.addLine("error: Robot instance is null");
            activeOpMode.telemetry.update();
        }
        return robot;
    }

    // This method can be called to register subsystems
    public void registerSubsystem(SubsystemType subsystem) {
        availableSubsystems.add(subsystem);
        activeOpMode.telemetry.addLine("Registered subsystem: " + subsystem);
        activeOpMode.telemetry.update();
    }

    // Check if the subsystem exists
    public boolean hasSubsystem(SubsystemType subsystem) {
        if (!availableSubsystems.contains(subsystem)) {
            activeOpMode.telemetry.addLine("Warning: Subsystem " + subsystem + " is not available on this robot.");
            activeOpMode.telemetry.update();
            return false;
        }
        return true;
    }

    // Initialize teleop or autonomous, depending on which is used
    public void init(OpModeType oType){
        driverStationTelemetryManager = new DriverStationTelemetryManager(activeOpMode.telemetry);
        opModeType = oType;
            if (opModeType == OpModeType.TELEOP) {
                initTele();
            } else {
                initAuto();
            }
        }

    private void initTele() {
        switch (robotType) {
            case CHASSIS_19429_HUB_TWO_DEAD_WHEELS:
            case CHASSIS_19429_A_PINPOINT: {
                mecanumDriveSubsystem.init();
                break;
            }
            case CENTERSTAGE_OTOS:


            case CENTERSTAGE_HUB_TWO_DEAD_WHEELS:
            {
                visionSubsystem.init();
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
            case CHASSIS_19429_HUB_TWO_DEAD_WHEELS:
            case CHASSIS_19429_A_PINPOINT:
                mecanumDriveSubsystem.init();
                break;

            case CENTERSTAGE_OTOS:
            case CENTERSTAGE_HUB_TWO_DEAD_WHEELS:{
                visionSubsystem.init();
                mecanumDriveSubsystem.init();
                intakeSubsystem.init();
                gripperSubsystem.init();
                liftSubsystem.init();
                shoulderSubsystem.init();
                break;
            }
        }
    }

    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public IntakeSubsystem getIntakeSubsystem()  {return intakeSubsystem;}
    public GripperSubsystem getEndEffectorSubsystem()  {return gripperSubsystem;}
    public LiftSubsystem getLiftSubsystem()  {return liftSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}
    public ClimberSubsystem getClimberSubsystem(){return climberSubsystem;}
    public DriverStationTelemetryManager getDriverStationTelemetryManager() {return driverStationTelemetryManager;
    }

    public static RobotType getPreviousRobotType(RobotType currentType) {
        RobotType[] types = RobotType.values();
        int index = currentType.ordinal() - 1;
        if (index < 0) {
            index = types.length - 1;  // Wrap around to the last enum value if we're at the first one
        }
        return types[index];
    }

    public static RobotType getNextRobotType(RobotType currentType) {
        RobotType[] types = RobotType.values();
        int index = (currentType.ordinal() + 1) % types.length;  // Wrap around to the first enum value if we're at the last one
        return types[index];
    }

    public boolean isAutoMode() {
        return opModeType == OpModeType.AUTO;
    }

    public boolean isTeleOpMode() {
        return opModeType == OpModeType.TELEOP;
    }

}






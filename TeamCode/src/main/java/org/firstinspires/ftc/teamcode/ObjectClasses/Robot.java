package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.Vision.VisionSubsystem;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class Robot {

    private static Robot robot = null;

    private DriverStationTelemetryManager driverStationTelemetryManager;

    public RobotType robotType;
    public OpModeType opModeType;
    public enum RobotType {
        CHASSIS_19429_B_PINPOINT,
        CHASSIS_19429_B_HUB_TWO_DEAD_WHEELS,
        INTO_THE_DEEP,
        LIFT_BOT_PINPOINT,
        INTAKE_TESTER
    }
    public enum OpModeType {TELEOP, AUTO}

    private static LinearOpMode activeOpMode;

    private static DriveSubsystem mecanumDriveSubsystem;

    //Sample Subsystems
    private static SampleIntakeSubsystem sampleIntakeSubsystem;
    private static SampleLiftSubsystem sampleLiftSubsystem;
    private static SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem;

    //Specimen Subsystems
    private static ShoulderSubsystem shoulderSubsystem;
    private static GripperSubsystem gripperSubsystem;

    private static ClimberSubsystem climberSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static SampleHandlingStateMachine sampleHandlingStateMachine;


    public enum SubsystemType {
        DRIVE, SAMPLE_INTAKE, SAMPLE_ACTUATOR, SAMPLE_LIFT, SPECIMEN_GRIPPER, SPECIMEN_ARM, CLIMBER, VISION
    }

    // Use an EnumSet for tracking available subsystems
    private final Set<SubsystemType> availableSubsystems = EnumSet.noneOf(SubsystemType.class);
    private final Map<SubsystemType, Object> subsystemMap = new HashMap<>();


    /* Constructor */
    private Robot(LinearOpMode opMode, RobotType rType) {
        activeOpMode = opMode;
        robotType = rType;
        HardwareMap hMap = opMode.hardwareMap;
        CreateSubsystems(hMap);
    }

    private void CreateSubsystems(HardwareMap hardwareMap) {

        switch (robotType) {
            case INTO_THE_DEEP: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, robotType);
                registerSubsystem(SubsystemType.DRIVE, mecanumDriveSubsystem);

//              sampleIntakeSubsystem = new SampleIntakeSubsystem(hardwareMap, "sampleintake", "colorsensor"); // If we have a color sensor
                sampleIntakeSubsystem = new SampleIntakeSubsystem(hardwareMap, "sampleintake");
                registerSubsystem(SubsystemType.SAMPLE_INTAKE, sampleIntakeSubsystem);

                sampleLiftSubsystem = new SampleLiftSubsystem(hardwareMap, "samplelift");
                registerSubsystem(SubsystemType.SAMPLE_LIFT, sampleLiftSubsystem);

                // sampleLinearActuatorSubsystem = new SampleLinearActuatorSubsystem(hardwareMap, "samplelinearactuator", "actuatorlimitSwitch");
                sampleLinearActuatorSubsystem = new SampleLinearActuatorSubsystem(hardwareMap, "samplelinearactuator");
                registerSubsystem(SubsystemType.SAMPLE_ACTUATOR, sampleLinearActuatorSubsystem);
                break;
            }

            case INTAKE_TESTER: {
                sampleIntakeSubsystem = new SampleIntakeSubsystem(hardwareMap, "sampleintake");
                registerSubsystem(SubsystemType.SAMPLE_INTAKE, sampleIntakeSubsystem);

                sampleLiftSubsystem = new SampleLiftSubsystem(hardwareMap, "samplelift");
                registerSubsystem(SubsystemType.SAMPLE_LIFT, sampleLiftSubsystem);

                sampleLinearActuatorSubsystem = new SampleLinearActuatorSubsystem(hardwareMap, "samplelinearactuator");
                registerSubsystem(SubsystemType.SAMPLE_ACTUATOR, sampleLinearActuatorSubsystem);
                break;
            }

            //Just the drive base
            case CHASSIS_19429_B_HUB_TWO_DEAD_WHEELS:
            case CHASSIS_19429_B_PINPOINT: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, robotType);
                registerSubsystem(SubsystemType.DRIVE, mecanumDriveSubsystem);
                break;
            }

            case LIFT_BOT_PINPOINT: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, robotType);
                registerSubsystem(SubsystemType.DRIVE, mecanumDriveSubsystem);

//                sampleLiftSubsystem = new SampleLiftSubsystem(hardwareMap, "samplelift");
//                registerSubsystem(SubsystemType.SAMPLE_LIFT);
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
    public void registerSubsystem(SubsystemType type, Object subsystem) {
        subsystemMap.put(type, subsystem);
        availableSubsystems.add(type);
        activeOpMode.telemetry.addLine("Registered subsystem: " + subsystem);
    }

    // Check if the subsystem exists
    public boolean hasSubsystem(SubsystemType subsystem) {
        if (!availableSubsystems.contains(subsystem)) {
            activeOpMode.telemetry.addLine(subsystem + " not available");
            return false;
        }
        return true;
    }

    // Initialize shared subsystems for both TeleOp and Autonomous
    public void init(OpModeType oType) {
        driverStationTelemetryManager = new DriverStationTelemetryManager(activeOpMode.telemetry);
        opModeType = oType;

        initRegisteredSubsystems();

        // If specific TeleOp or Auto differences arise later, you can re-separate here.
        // For example, if you add vision later, you could add something like:
         if (opModeType == OpModeType.TELEOP
                 && hasSubsystem(SubsystemType.SAMPLE_INTAKE)
                 && hasSubsystem(SubsystemType.SAMPLE_LIFT)
                 && hasSubsystem(SubsystemType.SAMPLE_ACTUATOR)) {
             sampleHandlingStateMachine = new SampleHandlingStateMachine(sampleLinearActuatorSubsystem, sampleIntakeSubsystem, sampleLiftSubsystem);
         }
    }

    // Common initialization method for all modes
    private void initRegisteredSubsystems() {
        for (SubsystemType type : availableSubsystems) {
            Object subsystem = subsystemMap.get(type);
            try {
                assert subsystem != null;
                subsystem.getClass().getMethod("init").invoke(subsystem);
            } catch (Exception e) {
                activeOpMode.telemetry.addLine("Failed to initialize subsystem: " + type);
            } finally {
                activeOpMode.telemetry.update();
            }
        }
    }

    public SampleLiftSubsystem getSampleLiftSubsystem()  {return sampleLiftSubsystem;}
    public SampleIntakeSubsystem getSampleIntakeSubsystem()  {return sampleIntakeSubsystem;}
    public SampleLinearActuatorSubsystem getSampleLinearActuatorSubsystem()  {return sampleLinearActuatorSubsystem;}

    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}
    public DriverStationTelemetryManager getDriverStationTelemetryManager() {return driverStationTelemetryManager;}

    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public GripperSubsystem getEndEffectorSubsystem()  {return gripperSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public ClimberSubsystem getClimberSubsystem(){return climberSubsystem;}
    public SampleHandlingStateMachine getSampleHandlingStateMachine(){return sampleHandlingStateMachine;}

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






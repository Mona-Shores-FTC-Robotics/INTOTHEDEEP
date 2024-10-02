package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import static com.example.sharedconstants.FieldConstants.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MatchConfig {

    public static AllianceColor finalAllianceColor = AllianceColor.RED;
    public static SideOfField finalSideOfField = SideOfField.OBSERVATION;
    public static Robot.RobotType finalRobotType = Robot.RobotType.ROBOT_CHASSIS_TWO_DEAD_WHEEL_INTERNAL_IMU;  // Default to an initial type

    public static boolean verboseMode = false;

    public static Pose2d endOfAutonomousPose = null;
    public static double endOfAutonomousOffset;
    public static double endOfAutonomousAbsoluteYawDegrees;

    public static ElapsedTime teleOpTimer;
    public static ElapsedTime loopTimer;
    public static ElapsedTime timestampTimer;

    public static TelemetryPacket telemetryPacket;
}

package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MatchConfig {
    public enum AllianceColor {BLUE, RED}
    public enum SideOfField {BACKSTAGE, AUDIENCE}
    public enum TeamPropLocation {LEFT, CENTER, RIGHT}

    public static AllianceColor finalAllianceColor = AllianceColor.BLUE;
    public static SideOfField finalSideOfField = SideOfField.BACKSTAGE;
    public static TeamPropLocation finalTeamPropLocation = TeamPropLocation.CENTER;

    public static Pose2d endOfAutonomousPose = null;
    public static double endOfAutonomousOffset;
    public static double endOfAutonomousRelativeYawDegrees;
    public static double endOfAutonomousAbsoluteYawDegrees;
    public static boolean autoHasRun=false;

    public static ElapsedTime teleOpTimer;
    public static ElapsedTime loopTimer;
    public static ElapsedTime timestampTimer;

    public static TelemetryPacket telemetryPacket;



}

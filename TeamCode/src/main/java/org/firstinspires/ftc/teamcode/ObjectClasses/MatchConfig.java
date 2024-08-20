package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class MatchConfig {
    public static InitVisionProcessor.AllianceColor finalAllianceColor = InitVisionProcessor.AllianceColor.BLUE;
    public static InitVisionProcessor.SideOfField finalSideOfField = InitVisionProcessor.SideOfField.BACKSTAGE;
    public static InitVisionProcessor.TeamPropLocation finalTeamPropLocation = InitVisionProcessor.TeamPropLocation.CENTER;
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

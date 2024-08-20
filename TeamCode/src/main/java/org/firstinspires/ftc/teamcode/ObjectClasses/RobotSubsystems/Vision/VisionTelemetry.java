package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class VisionTelemetry {

    public static void telemetryForInitProcessing(GamepadHandling gamepadHandling) {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        telemetry.addData("Alliance Color", MatchConfig.finalAllianceColor);
        telemetry.addData("Side of the Field", MatchConfig.finalSideOfField);
        telemetry.addData("Team Prop Location",  MatchConfig.finalTeamPropLocation);
        telemetry.addLine("");
        telemetry.addData("Left Square Blue/Red Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getLeftPercent(), 4, 1));
        telemetry.addData("Middle Square Blue/Red Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getCenterPercent(), 4, 1));
        telemetry.addData("Right Square Blue/Red Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getRightPercent(), 4, 1));

        telemetry.addData("Total Red", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentRedTotal, 4, 1));
        telemetry.addData("Total Blue", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentBlueTotal, 4, 1));
        telemetry.addData("Alliance Color Problem Flag", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColorDeterminationProblem);

        telemetry.addData("Stage Door Left Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentLeftStageDoorZone, 4, 1));
        telemetry.addData("Stage Door Right Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentRightStageDoorZone, 4, 1));

    }

}

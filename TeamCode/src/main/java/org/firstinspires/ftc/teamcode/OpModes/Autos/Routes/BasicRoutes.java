package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static com.example.sharedconstants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.SideOfField.*;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses.MecanumDriveMona;

public class BasicRoutes {
    //Variables to store routes for all four start locations
    public static Action redAudienceBotRoute;
    public static Action redBackstageBotRoute;
    public static Action blueBackstageBotRoute;
    public static Action blueAudienceBotRoute;

    public static void BuildRoutes() {

        MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        /** BLUE BACKSTAGE **/
        PosesForBasicRoutes blueBackstageRightPoses = new PosesForBasicRoutes(BLUE, BACKSTAGE);
        blueBackstageBotRoute = mecanumDrive.actionBuilder(blueBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueBackstageRightPoses))
                .build();

        /** RED BACKSTAGE **/
        PosesForBasicRoutes redBackstageRightPoses = new PosesForBasicRoutes(RED, BACKSTAGE);
        redBackstageBotRoute = mecanumDrive.actionBuilder(redBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redBackstageRightPoses))
                .build();

        /** BLUE AUDIENCE **/
        PosesForBasicRoutes blueAudienceRightPoses = new PosesForBasicRoutes(BLUE, AUDIENCE);
        blueAudienceBotRoute = mecanumDrive.actionBuilder(blueAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueAudienceRightPoses))
                .build();

        /** RED AUDIENCE **/
        PosesForBasicRoutes redAudienceRightPoses = new PosesForBasicRoutes(RED, AUDIENCE);
        redAudienceBotRoute = mecanumDrive.actionBuilder(redAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redAudienceRightPoses))
                .build();
    }


    public static class RouteBuilder {
        MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        public Action ExampleRoute(PosesForBasicRoutes posesForRoute) {
            return mecanumDrive.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().FirstWaypoint(posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
        }

        private Action FirstWaypoint(PosesForBasicRoutes posesForRoute) {
            return mecanumDrive.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.waypointPose, posesForRoute.waypointPose.heading.log())
                    .build();
        }

        private Action Park(PosesForBasicRoutes posesForRoute) {
            return mecanumDrive.actionBuilder(posesForRoute.waypointPose)
                    .strafeTo(PoseToVector(posesForRoute.parkPose))
                    .turnTo(posesForRoute.parkOrientation)
                    .build();
        }
    }

    public static Action getRoute(MatchConfig.AllianceColor allianceColor, MatchConfig.SideOfField sideOfField) {
        if (allianceColor == MatchConfig.AllianceColor.BLUE && sideOfField == MatchConfig.SideOfField.BACKSTAGE){
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = BLUE_BACKSTAGE_START_POSE;
            return blueBackstageBotRoute;
        } else if (allianceColor == MatchConfig.AllianceColor.BLUE && sideOfField == MatchConfig.SideOfField.AUDIENCE){
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose  = BLUE_AUDIENCE_START_POSE;
            return blueAudienceBotRoute;
        } else if (allianceColor == MatchConfig.AllianceColor.RED && sideOfField == MatchConfig.SideOfField.BACKSTAGE){
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = RED_BACKSTAGE_START_POSE;
            return redBackstageBotRoute;
        } else {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose  = RED_AUDIENCE_START_POSE;
            return redAudienceBotRoute;
        }
    }
}


package com.example.sharedconstants.Routes.FunctionalRoutes;

import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.FieldConstants.AllianceColor.*;
import static com.example.sharedconstants.FieldConstants.SideOfField.*;
import static com.example.sharedconstants.FieldConstants.TeamPropLocation.*;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;

public class FunctionalRoutesExample extends Routes {
    public FunctionalRoutesExample(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    //Variables to store routes for team prop left for all four start locations
    public Action redAudienceBotTeamPropLeftRoute;
    public Action redBackstageBotTeamPropLeftRoute;
    public Action blueBackstageBotTeamPropLeftRoute;
    public Action blueAudienceBotTeamPropLeftRoute;

    //Variables to store routes for team prop center for all four start locations
    public Action redAudienceBotTeamPropCenterRoute;
    public Action redBackstageBotTeamPropCenterRoute;
    public Action blueBackstageBotTeamPropCenterRoute;
    public Action blueAudienceBotTeamPropCenterRoute;

    //Variables to store routes for team prop right for all four start locations
    public Action redAudienceBotTeamPropRightRoute;
    public Action redBackstageBotTeamPropRightRoute;
    public Action blueBackstageBotTeamPropRightRoute;
    public Action blueAudienceBotTeamPropRightRoute;

    public void BuildRoutes() {

    /** RED AUDIENCE ROUTES **/
        PosesForFunctionalRoutesExample redAudienceLeftPoses = new PosesForFunctionalRoutesExample(RED, AUDIENCE, LEFT);
        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(redAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redAudienceLeftPoses))
                .build();

        PosesForFunctionalRoutesExample redAudienceCenterPoses = new PosesForFunctionalRoutesExample(RED, AUDIENCE, CENTER);
        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(redAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redAudienceCenterPoses))
                .build();

        PosesForFunctionalRoutesExample redAudienceRightPoses = new PosesForFunctionalRoutesExample(RED, AUDIENCE, RIGHT);
        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(redAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redAudienceRightPoses))
                .build();

        /** RED BACKSTAGE ROUTES **/
        PosesForFunctionalRoutesExample redBackstageLeftPoses = new PosesForFunctionalRoutesExample(RED, BACKSTAGE, LEFT);
        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(redBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redBackstageLeftPoses))
                .build();

        PosesForFunctionalRoutesExample redBackstageCenterPoses = new PosesForFunctionalRoutesExample(RED, BACKSTAGE, CENTER);
        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(redBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redBackstageCenterPoses))
                .build();

        PosesForFunctionalRoutesExample redBackstageRightPoses = new PosesForFunctionalRoutesExample(RED, BACKSTAGE, RIGHT);
        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(redBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redBackstageRightPoses))
                .build();


        /** BLUE BACKSTAGE ROUTES **/
        PosesForFunctionalRoutesExample blueBackstageLeftPoses = new PosesForFunctionalRoutesExample(BLUE, BACKSTAGE, LEFT);
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(blueBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueBackstageLeftPoses))
                .build();

        PosesForFunctionalRoutesExample blueBackstageCenterPoses = new PosesForFunctionalRoutesExample(BLUE, BACKSTAGE, CENTER);
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(blueBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueBackstageCenterPoses))
                .build();

        PosesForFunctionalRoutesExample blueBackstageRightPoses = new PosesForFunctionalRoutesExample(BLUE, BACKSTAGE, RIGHT);
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(blueBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueBackstageRightPoses))
                .build();

        /** BLUE AUDIENCE ROUTES **/
        PosesForFunctionalRoutesExample blueAudienceLeftPoses = new PosesForFunctionalRoutesExample(BLUE, AUDIENCE, LEFT);
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(blueAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueAudienceLeftPoses))
                .build();

        PosesForFunctionalRoutesExample blueAudienceCenterPoses = new PosesForFunctionalRoutesExample(BLUE, AUDIENCE, CENTER);
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(blueAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueAudienceCenterPoses))
                .build();

        PosesForFunctionalRoutesExample blueAudienceRightPoses = new PosesForFunctionalRoutesExample(BLUE, AUDIENCE, RIGHT);
        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(blueAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueAudienceRightPoses))
                .build();


    }
    @Override
    public Action getBlueBackstageBotTeamPropLeftRoute() {
        return blueBackstageBotTeamPropLeftRoute;
    }

    @Override
    public Action getBlueAudienceBotTeamPropLeftRoute() {
        return blueAudienceBotTeamPropLeftRoute;
    }

    @Override
    public Action getBlueBackstageBotTeamPropCenterRoute() {
        return blueBackstageBotTeamPropCenterRoute;
    }

    @Override
    public Action getBlueAudienceBotTeamPropCenterRoute() {
        return blueAudienceBotTeamPropCenterRoute;
    }

    @Override
    public Action getBlueBackstageBotTeamPropRightRoute() {
        return blueBackstageBotTeamPropRightRoute;
    }

    @Override
    public Action getBlueAudienceBotTeamPropRightRoute() {
        return blueAudienceBotTeamPropRightRoute;
    }

    @Override
    public Action getRedBackstageBotTeamPropLeftRoute() {
        return redBackstageBotTeamPropLeftRoute;
    }

    @Override
    public Action getRedAudienceBotTeamPropLeftRoute() {
        return redAudienceBotTeamPropLeftRoute;
    }

    @Override
    public Action getRedBackstageBotTeamPropCenterRoute() {
        return redBackstageBotTeamPropCenterRoute;
    }

    @Override
    public Action getRedAudienceBotTeamPropCenterRoute() {
        return redAudienceBotTeamPropCenterRoute;
    }

    @Override
    public Action getRedBackstageBotTeamPropRightRoute() {
        return redBackstageBotTeamPropRightRoute;
    }

    @Override
    public Action getRedAudienceBotTeamPropRightRoute() {
        return redAudienceBotTeamPropRightRoute;
    }

    public class RouteBuilder {

        public Action ExampleRoute(PosesForFunctionalRoutesExample posesForRoute) {
            return roadRunnerDrive.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().FirstWaypoint(posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
        }

        private Action FirstWaypoint(PosesForFunctionalRoutesExample posesForRoute) {
            return roadRunnerDrive.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.waypointPose, posesForRoute.waypointPose.heading.log())
                    .build();
        }

        private Action Park(PosesForFunctionalRoutesExample posesForRoute) {
            return roadRunnerDrive.actionBuilder(posesForRoute.waypointPose)
                    .strafeTo(PoseToVector(posesForRoute.parkPose))
                    .turnTo(posesForRoute.parkOrientation)
                    .build();
        }
    }
}


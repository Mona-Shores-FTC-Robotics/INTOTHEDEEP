package com.example.sharedconstants.Routes.FunctionalRoutes;

import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.FieldConstants.AllianceColor.*;
import static com.example.sharedconstants.FieldConstants.SideOfField.*;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;

public class FunctionalRoutesExample extends Routes {
    public FunctionalRoutesExample(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    // Variables to store routes for all four start locations (red/blue audience and backstage)
    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public void BuildRoutes() {

        /** RED AUDIENCE ROUTE **/
        PosesForFunctionalRoutesExample redAudiencePoses = new PosesForFunctionalRoutesExample(RED, AUDIENCE);
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(redAudiencePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redAudiencePoses))
                .build();

        /** RED BACKSTAGE ROUTE **/
        PosesForFunctionalRoutesExample redBackstagePoses = new PosesForFunctionalRoutesExample(RED, BACKSTAGE);
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(redBackstagePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redBackstagePoses))
                .build();

        /** BLUE AUDIENCE ROUTE **/
        PosesForFunctionalRoutesExample blueAudiencePoses = new PosesForFunctionalRoutesExample(BLUE, AUDIENCE);
        blueAudienceBotRoute = roadRunnerDrive.actionBuilder(blueAudiencePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueAudiencePoses))
                .build();

        /** BLUE BACKSTAGE ROUTE **/
        PosesForFunctionalRoutesExample blueBackstagePoses = new PosesForFunctionalRoutesExample(BLUE, BACKSTAGE);
        blueBackstageBotRoute = roadRunnerDrive.actionBuilder(blueBackstagePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueBackstagePoses))
                .build();
    }

    @Override
    public Action getBlueBackstageBotRoute() {
        return blueBackstageBotRoute;
    }

    @Override
    public Action getBlueAudienceBotRoute() {
        return blueAudienceBotRoute;
    }

    @Override
    public Action getRedBackstageBotRoute() {
        return redBackstageBotRoute;
    }

    @Override
    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
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


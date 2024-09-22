package com.example.sharedconstants.Routes.FunctionalRoutes;

import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.FieldConstants.AllianceColor.*;
import static com.example.sharedconstants.FieldConstants.SideOfField.*;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class FunctionalRoutesExample extends Routes {
    public FunctionalRoutesExample(RobotAdapter roadRunnerDrive) {
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
        redAudienceBotRoute = robotAdapter.actionBuilder(redAudiencePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redAudiencePoses))
                .build();

        /** RED BACKSTAGE ROUTE **/
        PosesForFunctionalRoutesExample redBackstagePoses = new PosesForFunctionalRoutesExample(RED, BACKSTAGE);
        redBackstageBotRoute = robotAdapter.actionBuilder(redBackstagePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(redBackstagePoses))
                .build();

        /** BLUE AUDIENCE ROUTE **/
        PosesForFunctionalRoutesExample blueAudiencePoses = new PosesForFunctionalRoutesExample(BLUE, AUDIENCE);
        blueAudienceBotRoute = robotAdapter.actionBuilder(blueAudiencePoses.startingPose)
                .stopAndAdd(new RouteBuilder().ExampleRoute(blueAudiencePoses))
                .build();

        /** BLUE BACKSTAGE ROUTE **/
        PosesForFunctionalRoutesExample blueBackstagePoses = new PosesForFunctionalRoutesExample(BLUE, BACKSTAGE);
        blueBackstageBotRoute = robotAdapter.actionBuilder(blueBackstagePoses.startingPose)
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
            return robotAdapter.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().FirstWaypoint(posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
        }

        private Action FirstWaypoint(PosesForFunctionalRoutesExample posesForRoute) {
            return robotAdapter.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.waypointPose, posesForRoute.waypointPose.heading.log())
                    .build();
        }

        private Action Park(PosesForFunctionalRoutesExample posesForRoute) {
            return robotAdapter.actionBuilder(posesForRoute.waypointPose)
                    .strafeTo(PoseToVector(posesForRoute.parkPose))
                    .turnTo(posesForRoute.parkOrientation)
                    .build();
        }
    }
}


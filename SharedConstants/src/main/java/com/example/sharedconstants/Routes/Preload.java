package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotAdapter;

public class Preload extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void BuildRoutes() {
        redBackstageBotRoute = robotAdapter.getActionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        blueAudienceBotRoute = robotAdapter.getActionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        redAudienceBotRoute = robotAdapter.getActionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_PRELOAD_RED_AUDIENCE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_AUDIENCE))
                .build();

        blueBackstageBotRoute = robotAdapter.getActionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_PRELOAD_RED_AUDIENCE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_AUDIENCE))
                .build();
    }

    @Override
    public Action getBlueBackstageBotRoute() {
        return blueBackstageBotRoute;
    }

    @Override
    public Action getBlueAudienceBotRoute() {
        return this.blueAudienceBotRoute;
    }

    @Override
    public Action getRedBackstageBotRoute() {
        return redBackstageBotRoute;
    }

    @Override
    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
    }
}

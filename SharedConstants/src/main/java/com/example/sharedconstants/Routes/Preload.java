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
        redBackstageBotRoute = robotAdapter.getActionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(OBSERVATION_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        blueAudienceBotRoute = robotAdapter.getActionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(OBSERVATION_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        redAudienceBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(NET_START_POSE, CHAMBER_PRELOAD_RED_AUDIENCE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_AUDIENCE))
                .build();

        blueBackstageBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(NET_START_POSE, CHAMBER_PRELOAD_RED_AUDIENCE))
                .stopAndAdd(new RouteBuilder().NullDriveAction(CHAMBER_PRELOAD_RED_AUDIENCE))
                .build();
    }

    @Override
    public Action getBlueNetBotRoute() {
        return blueBackstageBotRoute;
    }

    @Override
    public Action getBlueObservationBotRoute() {
        return this.blueAudienceBotRoute;
    }

    @Override
    public Action getRedObservationBotRoute() {
        return redBackstageBotRoute;
    }

    @Override
    public Action getRedNetBotRoute() {
        return redAudienceBotRoute;
    }
}

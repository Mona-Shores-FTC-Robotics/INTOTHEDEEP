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
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(OBSERVATION_START_POSE, OBS_CHAMBER_PRELOAD))
                .stopAndAdd(new RouteBuilder().NullDriveAction(OBS_CHAMBER_PRELOAD))
                .build();

        blueAudienceBotRoute = robotAdapter.getActionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(OBSERVATION_START_POSE, OBS_CHAMBER_PRELOAD))
                .stopAndAdd(new RouteBuilder().NullDriveAction(OBS_CHAMBER_PRELOAD))
                .build();

        redAudienceBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(NET_START_POSE, NET_CHAMBER_PRELOAD))
                .stopAndAdd(new RouteBuilder().NullDriveAction(NET_CHAMBER_PRELOAD))
                .build();

        blueBackstageBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(NET_START_POSE, NET_CHAMBER_PRELOAD))
                .stopAndAdd(new RouteBuilder().NullDriveAction(NET_CHAMBER_PRELOAD))
                .build();
    }

    @Override
    public Action getObservationBotRoute() {
        return redBackstageBotRoute;
    }

    @Override
    public Action getNetBotRoute() {
        return redAudienceBotRoute;
    }
}

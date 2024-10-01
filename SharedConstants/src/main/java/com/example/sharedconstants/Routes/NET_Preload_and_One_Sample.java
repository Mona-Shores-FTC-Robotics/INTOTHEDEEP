package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_PRELOAD_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_ZONE_RED_PICKUP;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_START_POSE;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotAdapter;

public class NET_Preload_and_One_Sample extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public static Action blueTestRoute;

    public NET_Preload_and_One_Sample(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        redBackstageBotRoute = robotAdapter.actionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(OBSERVATION_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_PRELOAD_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(OBSERVATION_ZONE_RED_PICKUP, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        blueAudienceBotRoute = robotAdapter.rotatedActionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(OBSERVATION_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        redAudienceBotRoute = robotAdapter.actionBuilder(NET_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(NET_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();

        blueBackstageBotRoute = robotAdapter.rotatedActionBuilder(NET_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(NET_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();
    }

    @Override
    public Action getBlueNetBotRoute() {
        return blueBackstageBotRoute;
    }

    @Override
    public Action getBlueObservationBotRoute() {
        return blueAudienceBotRoute;
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

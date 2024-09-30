package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_PRELOAD_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_BACKSTAGE2;
import static com.example.sharedconstants.FieldConstants.CHAMBER_TWO_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_ZONE_RED_PICKUP;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.example.sharedconstants.RobotAdapter;

public class Preload_and_One_Specimen extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public static Action blueTestRoute;

    public Preload_and_One_Specimen(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        redBackstageBotRoute = robotAdapter.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_PRELOAD_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(OBSERVATION_ZONE_RED_PICKUP, CHAMBER_TWO_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_TWO_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .build();

        blueAudienceBotRoute = robotAdapter.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .build();

        redAudienceBotRoute = robotAdapter.actionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();

        blueBackstageBotRoute = robotAdapter.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_RED_AUDIENCE))
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
}

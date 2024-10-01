package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.CHAMBER_PRELOAD_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_TWO_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_ZONE_RED_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_START_POSE;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotAdapter;

public class OBS_Preload_and_One_Specimen extends Routes {

    public Action redObservationBotRoute;
    public Action blueObservationBotRoute;

    public OBS_Preload_and_One_Specimen(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        redObservationBotRoute = robotAdapter.actionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(OBSERVATION_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_PRELOAD_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(OBSERVATION_ZONE_RED_PICKUP, CHAMBER_TWO_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_TWO_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .build();

        blueObservationBotRoute = robotAdapter.rotatedActionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(OBSERVATION_START_POSE, CHAMBER_PRELOAD_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_PRELOAD_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(new RouteBuilder().ScoreSpecimen(OBSERVATION_ZONE_RED_PICKUP, CHAMBER_TWO_RED_BACKSTAGE))
                .stopAndAdd(new RouteBuilder().PickupSpecimen(CHAMBER_TWO_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(new RouteBuilder().NullDriveAction(OBSERVATION_ZONE_RED_PICKUP))
                .build();
    }

    @Override
    public Action getBlueObservationBotRoute() {
        return blueObservationBotRoute;
    }

    @Override
    public Action getRedObservationBotRoute() {
        return redObservationBotRoute;
    }

}

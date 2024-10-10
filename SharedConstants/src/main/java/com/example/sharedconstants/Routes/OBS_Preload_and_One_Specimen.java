package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_PRELOAD;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_THREE_VEC;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_TWO;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_ZONE_RED_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_WAYPOINT;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotAdapter;

public class OBS_Preload_and_One_Specimen extends Routes {

    public OBS_Preload_and_One_Specimen(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        // Initialize RouteBuilder once
        RouteBuilder routeBuilder = new RouteBuilder();

        observationBotRoute = robotAdapter.getActionBuilder(OBSERVATION_START_POSE)
                .stopAndAdd(routeBuilder.ScorePreloadSpecimen(OBSERVATION_START_POSE, OBS_CHAMBER_PRELOAD))
                .stopAndAdd(routeBuilder.PickupSpecimen(OBS_CHAMBER_PRELOAD, OBS_WAYPOINT, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(routeBuilder.ScoreSpecimen(OBSERVATION_ZONE_RED_PICKUP, OBS_CHAMBER_TWO))
                .stopAndAdd(routeBuilder.PickupSpecimen(OBS_CHAMBER_TWO, OBS_WAYPOINT, OBSERVATION_ZONE_RED_PICKUP))
                .stopAndAdd(routeBuilder.NullDriveAction(OBSERVATION_ZONE_RED_PICKUP))
                .build();
    }
}

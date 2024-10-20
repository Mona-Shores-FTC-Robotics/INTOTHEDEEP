package com.example.sharedconstants.Routes.OBS.PushAndScore;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.CHAMBER_STAGING_FOR_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike extends OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike {

    public OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickupSpecimenFromWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToSplineHeading(CHAMBER_STAGING_FOR_PICKUP, ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_ZONE_PICKUP), ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL));
    }
}


package com.example.sharedconstants.Routes.OBS.PushAndScore;

import com.example.sharedconstants.RobotAdapter;

import static com.example.sharedconstants.FieldConstants.*;

public class OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike extends OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike {

    public OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.buildRoute();
        pushThirdNeutralSpecimen();
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickupSpecimenFromWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_ZONE_PICKUP), ANGLE_TOWARD_RED);
    }

    public void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToSplineHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_THREE), ANGLE_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_THREE), ANGLE_TOWARD_RED);
    }
}


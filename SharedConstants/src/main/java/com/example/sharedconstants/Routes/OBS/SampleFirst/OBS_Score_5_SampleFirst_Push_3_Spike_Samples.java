package com.example.sharedconstants.Routes.OBS.SampleFirst;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_STAGING_FOR_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_5_SampleFirst_Push_3_Spike_Samples extends OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade {

    public OBS_Score_5_SampleFirst_Push_3_Spike_Samples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute()
    {
        super.buildRoute();
        pushSecondNeutralSpecimen();
        pickupSpecimenFromWall(false);
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        pushThirdNeutralSpecimen();
        pickupSpecimenFromWall(false   );
        scoreOnHighChamber(CHAMBER_SLOT_THREE);
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

package com.example.sharedconstants.Routes.OBS.SampleFirst;

import static com.example.sharedconstants.FieldConstants.ANGLE_340_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_4_SampleFirst_Push_2_Spike_Samples extends OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade {

    public OBS_Score_4_SampleFirst_Push_2_Spike_Samples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pushSecondNeutralSpecimen();
        pickupSpecimenFromWall(false);
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_THREE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickupSpecimenFromWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_340_DEGREES)
                .splineToLinearHeading(OBS_ZONE_PICKUP, ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL));
    }


}


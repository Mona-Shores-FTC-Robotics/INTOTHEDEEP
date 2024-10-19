package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Push_1_Score_2_Specimens_Preload_And_1_Premade extends OBS_Score_1_Specimen_Preload {

    public OBS_Push_1_Score_2_Specimens_Preload_And_1_Premade(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute(){
        super.buildRoute();
        pushFirstNeutralSpecimen();
        pickupSpecimenFromWall(false);
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}


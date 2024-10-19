package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Push_3_Score_5_Specimens_Preload_And_2_Premade_And_2_Neutral extends OBS_Push_3_Score_4_Specimens_Preload_And_2_Premade_And_1_Neutral {

    public OBS_Push_3_Score_5_Specimens_Preload_And_2_Premade_And_2_Neutral(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute()
    {
        super.buildRoute();
        pickupSpecimenFromWall(true);
        scoreOnHighChamber(CHAMBER_SLOT_FIVE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}

package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SIX;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Push_3_Score_6_Specimens_Preload_And_2_Premade_And_3_Neutral extends OBS_Push_3_Score_5_Specimens_Preload_And_2_Premade_And_2_Neutral {

    public OBS_Push_3_Score_6_Specimens_Preload_And_2_Premade_And_3_Neutral(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute()
    {
        super.buildRoute();
        pickupSpecimenFromWall(true);
        scoreOnHighChamber(CHAMBER_SLOT_SIX);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}

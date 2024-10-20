package com.example.sharedconstants.Routes.OBS.PushAndScore;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike extends OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike {

    public OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute()
    {
        super.buildRoute();
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_FIVE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}

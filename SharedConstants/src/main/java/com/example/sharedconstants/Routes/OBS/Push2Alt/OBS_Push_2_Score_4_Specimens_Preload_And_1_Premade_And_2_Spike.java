package com.example.sharedconstants.Routes.OBS.Push2Alt;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.OBS.OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike;

public class OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike extends OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike {

    public OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pickupSpecimenFromWall(true);
        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}


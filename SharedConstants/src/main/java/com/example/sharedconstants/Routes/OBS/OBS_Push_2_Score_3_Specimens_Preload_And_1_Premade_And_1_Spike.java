package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike extends OBS_Push_1_Score_2_Specimens_Preload_And_1_Premade {

    public OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute(){
        super.buildRoute();
        pushSecondNeutralSpecimen();
        pickupSpecimenFromWall(false);
        scoreOnHighChamber(CHAMBER_SLOT_THREE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}


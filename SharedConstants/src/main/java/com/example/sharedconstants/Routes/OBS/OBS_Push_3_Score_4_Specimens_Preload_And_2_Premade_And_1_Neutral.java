package com.example.sharedconstants.Routes.OBS;

import com.example.sharedconstants.RobotAdapter;

import static com.example.sharedconstants.FieldConstants.*;

public class OBS_Push_3_Score_4_Specimens_Preload_And_2_Premade_And_1_Neutral extends OBS_Push_2_Score_3_Specimens_Preload_And_2_Premade {

    public OBS_Push_3_Score_4_Specimens_Preload_And_2_Premade_And_1_Neutral(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.buildRoute();
        pushThirdNeutralSpecimen();
        pickupSpecimenFromWall(false);
        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}


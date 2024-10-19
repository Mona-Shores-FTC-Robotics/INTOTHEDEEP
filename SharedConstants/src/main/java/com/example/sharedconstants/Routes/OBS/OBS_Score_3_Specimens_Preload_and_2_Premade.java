package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_3_Specimens_Preload_and_2_Premade extends OBS_Score_2_Specimens_Preload_and_1_Premade {

    public OBS_Score_3_Specimens_Preload_and_2_Premade(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        pickupSpecimenFromWall(true);
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        pickupSpecimenFromWall(true);
        scoreOnHighChamber(CHAMBER_SLOT_THREE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}

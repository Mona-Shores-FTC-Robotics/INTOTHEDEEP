package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_2_Specimens_Preload_and_1_Premade extends OBS_Score_1_Specimen_Preload {

    public OBS_Score_2_Specimens_Preload_and_1_Premade(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.scoreObservationPreload(CHAMBER_SLOT_ONE);
        pickupSpecimenFromWall(true);
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

}
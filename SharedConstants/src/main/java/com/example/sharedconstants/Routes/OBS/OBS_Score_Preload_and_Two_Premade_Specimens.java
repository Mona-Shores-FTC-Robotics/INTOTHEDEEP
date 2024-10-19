package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_Preload_and_Two_Premade_Specimens extends OBS_Score_Preload_and_One_Premade_Specimen {

    public OBS_Score_Preload_and_Two_Premade_Specimens(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_THREE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}

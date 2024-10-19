package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;

import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_Preload_Push_Two_Neutral_Specimens_and_Score_Two_Premade_Specimens extends OBS_Score_Preload_Push_One_Neutral_Score_One_Premade_Specimen {

    public OBS_Score_Preload_Push_Two_Neutral_Specimens_and_Score_Two_Premade_Specimens(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute(){
        super.buildRoute();
        pushFirstNeutralSpecimen();
        pickupSpecimenFromWall();
        scoreSpecimenOnChamber(CHAMBER_SLOT_TWO);
        pushSecondNeutralSpecimen();
        pickupSpecimenFromWall();
        scoreSpecimenOnChamber(CHAMBER_SLOT_THREE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pushSecondNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToLinearHeading(OBS_BEHIND_SPIKE_TWO, ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_DELIVER_SPIKE_TWO, ANGLE_TOWARD_RED);
    }
}


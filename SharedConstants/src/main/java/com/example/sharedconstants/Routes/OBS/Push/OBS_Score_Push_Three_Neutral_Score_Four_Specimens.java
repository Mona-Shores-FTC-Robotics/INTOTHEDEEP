package com.example.sharedconstants.Routes.OBS.Push;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.OBS.OBS_Score_Preload_Push_Two_Neutral_Specimens_and_Score_Two_Premade_Specimens;

import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;

public class OBS_Score_Push_Three_Neutral_Score_Four_Specimens extends OBS_Score_Preload_Push_Two_Neutral_Specimens_and_Score_Two_Premade_Specimens {

    public OBS_Score_Push_Three_Neutral_Score_Four_Specimens(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.buildRoute();
        pushSecondNeutralSpecimen();
        pickupSpecimenFromWall();
        scoreSpecimenOnChamber(CHAMBER_SLOT_THREE);
        pushThirdNeutralSpecimen();
        pickupSpecimenFromWall();
        scoreSpecimenOnChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    private void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(Math.toRadians(-10))
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(NEXT_TO_OBS_ASCENT), ANGLE_TOWARD_BLUE)
                .splineToLinearHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_DELIVER_SPIKE_THREE, ANGLE_TOWARD_RED);
    }
}


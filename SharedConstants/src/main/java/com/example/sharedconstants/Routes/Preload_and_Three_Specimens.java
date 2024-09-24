package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public class Preload_and_Three_Specimens extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public Preload_and_Three_Specimens(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void BuildRoutes() {

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30),
                new AngularVelConstraint(Math.PI / 2)
        ));

        /** RED AUDIENCE **/
        redAudienceBotRoute = robotAdapter.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_BEHIND_NEUTRAL_AUDIENCE_1_TJ,FACE_135_DEGREES) // Moves robot behind first sample
                .setReversed(false)
                .splineToConstantHeading(SPIKE_NEUTRAL_AUDIENCE_1_TJ,FACE_TOWARD_BLUE) // Moves robot to grab first sample
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(WALL_ALIGN_POS_AUDIENCE_TJ,FACE_TOWARD_BLUE) // Moves robot to grab first sample
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(NET_POS_AUDIENCE_TJ, FACE_TOWARD_RED)
                .waitSeconds(1)
                .splineToConstantHeading(SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ, FACE_TOWARD_BLUE)
                .waitSeconds(0.5)
                .splineToConstantHeading(SPIKE_NEUTRAL_AUDIENCE_2_TJ, FACE_TOWARD_BLUE)
                .waitSeconds(1)
                .splineToLinearHeading(WALL_ALIGN_POS_AUDIENCE_TJ,FACE_TOWARD_RED) // Moves robot to grab first sample
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(NET_POS_AUDIENCE_TJ, FACE_TOWARD_RED)
                .waitSeconds(1)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_3_POS_TJ , FACE_TOWARD_BLUE)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(WALL_ALIGN_POS_AUDIENCE_TJ,FACE_TOWARD_RED) // Moves robot to grab first sample
                .splineToLinearHeading(NET_POS_AUDIENCE_TJ, FACE_TOWARD_RED)
                .waitSeconds(1)
                .splineToLinearHeading(RED_AUDIENCE_START_POSE, FACE_TOWARD_RED)
                .build();

        /** BLUE BACKSTAGE - THIS SHOULD MATCH THE RED AUDIENCE PATH AND START LOCATION **/
        blueBackstageBotRoute = robotAdapter.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_BEHIND_NEUTRAL_AUDIENCE_1_TJ,FACE_135_DEGREES) // Moves robot behind first sample
                .setReversed(false)
                .splineToConstantHeading(SPIKE_NEUTRAL_AUDIENCE_1_TJ,FACE_TOWARD_BLUE) // Moves robot to grab first sample
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(WALL_ALIGN_POS_AUDIENCE_TJ,FACE_TOWARD_BLUE) // Moves robot to grab first sample
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(NET_POS_AUDIENCE_TJ, FACE_TOWARD_RED)
                .waitSeconds(1)
                .splineToConstantHeading(SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ, FACE_TOWARD_BLUE)
                .waitSeconds(0.5)
                .splineToConstantHeading(SPIKE_NEUTRAL_AUDIENCE_2_TJ, FACE_TOWARD_BLUE)
                .waitSeconds(1)
                .splineToLinearHeading(WALL_ALIGN_POS_AUDIENCE_TJ,FACE_TOWARD_RED) // Moves robot to grab first sample
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(NET_POS_AUDIENCE_TJ, FACE_TOWARD_RED)
                .waitSeconds(1)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_3_POS_TJ , FACE_TOWARD_BLUE)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(WALL_ALIGN_POS_AUDIENCE_TJ,FACE_TOWARD_RED) // Moves robot to grab first sample
                .splineToLinearHeading(NET_POS_AUDIENCE_TJ, FACE_TOWARD_RED)
                .waitSeconds(1)
                .splineToLinearHeading(RED_AUDIENCE_START_POSE, FACE_TOWARD_RED)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = robotAdapter.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_1_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE2, FACE_TOWARD_BLUE, baseVelConstraint)
                // next cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_2_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE3, FACE_TOWARD_BLUE, baseVelConstraint)
                //last cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_3_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE4, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();

        /** BLUE AUDIENCE THIS SHOULD MATCH THE RED BACKSTAGE PATH AND START LOCATION **/
        blueAudienceBotRoute = robotAdapter.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_1_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                // next cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_2_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                //last cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_3_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();

    }

    @Override
    public Action getBlueBackstageBotRoute() {
        return blueBackstageBotRoute;
    }

    @Override
    public Action getBlueAudienceBotRoute() {
        return blueAudienceBotRoute;
    }

    @Override
    public Action getRedBackstageBotRoute() {
        return redBackstageBotRoute;
    }

    @Override
    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
    }

}


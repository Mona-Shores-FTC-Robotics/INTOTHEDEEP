package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_BACKSTAGE_VECTOR;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_1_TJ;
import static com.example.sharedconstants.FieldConstants.SPIKE_RED_1_Vec;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public class BasicRoute extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public BasicRoute(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void BuildRoutes() {

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10),
                new AngularVelConstraint(Math.PI / 2)
        ));

        /** RED AUDIENCE **/
        redAudienceBotRoute = robotAdapter.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE, baseVelConstraint)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_NEUTRAL_AUDIENCE_1_TJ, FACE_TOWARD_AUDIENCE, baseVelConstraint)
                .build();

        /** BLUE BACKSTAGE - THIS SHOULD MATCH THE RED AUDIENCE PATH AND START LOCATION **/
        blueBackstageBotRoute = robotAdapter.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE, baseVelConstraint)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_NEUTRAL_AUDIENCE_1_TJ, FACE_TOWARD_AUDIENCE, baseVelConstraint)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = robotAdapter.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_RED_1_Vec, FACE_TOWARD_BACKSTAGE, baseVelConstraint)
                .build();

        /** BLUE AUDIENCE THIS SHOULD MATCH THE RED BACKSTAGE PATH AND START LOCATION **/
        blueAudienceBotRoute = robotAdapter.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VECTOR, FACE_TOWARD_BLUE, baseVelConstraint)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_RED_1_Vec, FACE_TOWARD_BACKSTAGE, baseVelConstraint)
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


package com.example.sharedconstants.Routes.FunctionalRoutes;

import static com.example.sharedconstants.FieldConstants.ASCENT_RED_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.ASCENT_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.NET_ZONE_RED;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_RED_ZONE;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RUNG_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_1;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_2;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_3;
import static com.example.sharedconstants.FieldConstants.TANGENT_315_DEGREES;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;

public class dhsExample extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public dhsExample(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }



    public void BuildRoutes() {
        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_1, FACE_TOWARD_RED)
                .waitSeconds(2)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_2, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_3, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(ASCENT_RED_AUDIENCE, FACE_TOWARD_AUDIENCE)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(RUNG_RED_BACKSTAGE, FACE_TOWARD_BLUE)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED)
                .waitSeconds(2)
                .splineToConstantHeading(RUNG_RED_BACKSTAGE, FACE_TOWARD_BLUE)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED)
                .waitSeconds(2)
                .splineToConstantHeading(RUNG_RED_BACKSTAGE, FACE_TOWARD_BLUE)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED)
                .waitSeconds(2)
                .splineToLinearHeading(ASCENT_RED_BACKSTAGE, FACE_TOWARD_AUDIENCE)
                .build();

        /** BLUE AUDIENCE **/
        blueAudienceBotRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .build();

        /** BLUE BACKSTAGE **/
        blueBackstageBotRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
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


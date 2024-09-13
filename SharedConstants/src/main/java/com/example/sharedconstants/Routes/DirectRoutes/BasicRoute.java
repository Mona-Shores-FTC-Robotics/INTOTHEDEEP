package com.example.sharedconstants.Routes.DirectRoutes;

import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.FACE_135_DEGREES;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.NET_POS_AUDIENCE_TJ;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_ASCENT_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_RED_ZONE;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RIGHT_TO_CHAMBER;
import static com.example.sharedconstants.FieldConstants.SPIKE_BEHIND_NEUTRAL_AUDIENCE_1_TJ;
import static com.example.sharedconstants.FieldConstants.SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_1;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_1_TJ;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_2_TJ;
import static com.example.sharedconstants.FieldConstants.SPIKE_NEUTRAL_AUDIENCE_3_POS_TJ;
import static com.example.sharedconstants.FieldConstants.SPIKE_RED_1_OB;
import static com.example.sharedconstants.FieldConstants.SPIKE_RED_2_OB;
import static com.example.sharedconstants.FieldConstants.SPIKE_RED_3_OB;
import static com.example.sharedconstants.FieldConstants.WALL_ALIGN_POS_AUDIENCE_TJ;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;

public class BasicRoute extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public BasicRoute(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10),
                new AngularVelConstraint(Math.PI / 2)
        ));

        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();
        /** BLUE BACKSTAGE - THIS SHOULD MATCH THE RED AUDIENCE PATH AND START LOCATION **/
        blueBackstageBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();

        /** BLUE AUDIENCE THIS SHOULD MATCH THE RED BACKSTAGE PATH AND START LOCATION **/
        blueAudienceBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
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


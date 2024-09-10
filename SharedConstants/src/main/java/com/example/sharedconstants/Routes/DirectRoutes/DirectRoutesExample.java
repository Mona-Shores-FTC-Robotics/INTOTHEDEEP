package com.example.sharedconstants.Routes.DirectRoutes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;

public class DirectRoutesExample extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public DirectRoutesExample(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {


        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI / 2)
        ));

        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE)
                .build();

        /** BLUE BACKSTAGE - THIS SHOULD MATCH THE RED AUDIENCE PATH AND START LOCATION **/
        blueBackstageBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_1, FACE_TOWARD_BLUE)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(0.1)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_1_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
               // next cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(0.1)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_2_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                //last cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(0.1)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_3_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
               .splineToConstantHeading(CHAMBER_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();

        /** BLUE AUDIENCE THIS SHOULD MATCH THE RED BACKSTAGE PATH AND START LOCATION **/
        blueAudienceBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE)
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


package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotDriveAdapter;

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
                new TranslationalVelConstraint(30),
                new AngularVelConstraint(Math.PI / 2)
        ));

        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
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
        blueBackstageBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
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
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                //TODO Autonomous Learning: experiment with removing this wait. What is the difference?
                .waitSeconds(.1)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_1_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
               // next cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_2_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
                //last cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_3_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
               .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
                .build();

        /** BLUE AUDIENCE THIS SHOULD MATCH THE RED BACKSTAGE PATH AND START LOCATION **/
        blueAudienceBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                //TODO Autonomous Learning: experiment with removing this wait. What is the difference?
                .waitSeconds(.1)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_1_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
                // next cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_2_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
                //last cycle
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(RIGHT_TO_CHAMBER, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToConstantHeading(NEXT_TO_ASCENT_RED_BACKSTAGE, FACE_TOWARD_BLUE, baseVelConstraint)
                .splineToLinearHeading(SPIKE_RED_3_OB, FACE_TOWARD_RED, baseVelConstraint)
                .splineToConstantHeading(OBSERVATION_RED_ZONE, FACE_TOWARD_RED, baseVelConstraint)
                .waitSeconds(1)
                .splineToConstantHeading(CHAMBER_RED_BACKSTAGE_VEC, FACE_TOWARD_BLUE, baseVelConstraint)
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


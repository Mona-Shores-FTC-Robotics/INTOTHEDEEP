package com.example.sharedconstants.Routes.DirectRoutes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.FunctionalRoutes.FunctionalRoutesExample;
import com.example.sharedconstants.Routes.FunctionalRoutes.PosesForFunctionalRoutesExample;
import com.example.sharedconstants.Routes.Routes;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryAction;
import com.noahbres.meepmeep.roadrunner.entity.TurnAction;

import java.util.Arrays;
import java.util.List;

public class DirectRoutesExample extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public DirectRoutesExample(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
          //      .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE)
                // BASKET_RED_AUDIENCE_2_TJ
                // SPIKE_NEUTRAL_AUDIENCE_1_TJ
                // SPIKE_NEUTRAL_AUDIENCE_2_TJ
                // SPIKE_NEUTRAL_AUDIENCE_3_TJ
                .splineToLinearHeading(CHAMBER_RED_AUDIENCE, FACE_TOWARD_BLUE)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(SPIKE_BEHIND_NEUTRAL_AUDIENCE_1_TJ,FACE_135_DEGREES) // Moves robot behind first sample
//                .strafeTo(SPIKE_NEUTRAL_AUDIENCE_1_TJ) // Grabs first sample
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
                .build();
               // .strafeTo(WALL_ALIGN_POS_AUDIENCE_TJ)
        /** BLUE BACKSTAGE - THIS SHOULD MATCH THE RED AUDIENCE PATH AND START LOCATION **/
        blueBackstageBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_1, FACE_TOWARD_BLUE)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToConstantHeading(RUNG_RED_BACKSTAGE, FACE_TOWARD_BLUE)
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


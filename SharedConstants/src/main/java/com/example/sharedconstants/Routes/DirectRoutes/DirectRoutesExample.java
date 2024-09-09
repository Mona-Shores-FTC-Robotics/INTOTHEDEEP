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
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryAction;
import com.noahbres.meepmeep.roadrunner.entity.TurnAction;

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

        /** BLUE BACKSTAGE **/
        blueBackstageBotRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(SPIKE_NEUTRAL_BACKSTAGE_1, FACE_TOWARD_RED)
                .waitSeconds(2)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(SPIKE_NEUTRAL_BACKSTAGE_2, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(SPIKE_NEUTRAL_BACKSTAGE_3, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .waitSeconds(2)
                .splineToLinearHeading(ASCENT_BLUE_BACKSTAGE, FACE_TOWARD_AUDIENCE)
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RUNG_RED_BACKSTAGE, FACE_TOWARD_BLUE)
//                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
//                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_2, TANGENT_315_DEGREES)
//                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
//                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_3, TANGENT_315_DEGREES)
//                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
//                .strafeTo(PoseToVector(ASCENT_RED_AUDIENCE))
                .build();

        /** BLUE AUDIENCE **/
        blueAudienceBotRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(RUNG_BLUE_AUDIENCE, FACE_TOWARD_RED)
//                .splineToLinearHeading(OBSERVATION_BLUE_ZONE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(RUNG_BLUE_AUDIENCE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(OBSERVATION_BLUE_ZONE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(RUNG_BLUE_AUDIENCE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(OBSERVATION_BLUE_ZONE, TANGENT_225_DEGREES)
                .build();

        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_1, FACE_TOWARD_BLUE)
//                .splineToLinearHeading(OBSERVATION_RED_ZONE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(RUNG_RED_BACKSTAGE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(OBSERVATION_RED_ZONE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(RUNG_RED_BACKSTAGE, TANGENT_225_DEGREES)
//                .splineToLinearHeading(OBSERVATION_RED_ZONE, TANGENT_225_DEGREES)
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


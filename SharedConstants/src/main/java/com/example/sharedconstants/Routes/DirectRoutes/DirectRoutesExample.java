package com.example.sharedconstants.Routes.DirectRoutes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.example.sharedconstants.RobotDriveAdapter;
import com.example.sharedconstants.Routes.Routes;

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
                .splineToLinearHeading(SPIKE_NEUTRAL_BACKSTAGE_1, TANGENT_315_DEGREES)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .splineToLinearHeading(SPIKE_NEUTRAL_BACKSTAGE_2, TANGENT_315_DEGREES)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .splineToLinearHeading(SPIKE_NEUTRAL_BACKSTAGE_3, TANGENT_315_DEGREES)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .strafeTo(PoseToVector(ASCENT_BLUE_BACKSTAGE))
                .build();

        /** RED BACKSTAGE **/
        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_1, TANGENT_315_DEGREES)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_2, TANGENT_315_DEGREES)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .splineToLinearHeading(SPIKE_NEUTRAL_AUDIENCE_3, TANGENT_315_DEGREES)
                .splineToLinearHeading(NET_ZONE_RED, TANGENT_315_DEGREES)
                .strafeTo(PoseToVector(ASCENT_RED_AUDIENCE))
                .build();

        /** BLUE AUDIENCE **/
        blueAudienceBotRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(RUNG_BLUE_AUDIENCE, TANGENT_225_DEGREES)
                .splineToLinearHeading(OBSERVATION_BLUE_ZONE, TANGENT_225_DEGREES)
                .splineToLinearHeading(RUNG_BLUE_AUDIENCE, TANGENT_225_DEGREES)
                .splineToLinearHeading(OBSERVATION_BLUE_ZONE, TANGENT_225_DEGREES)
                .splineToLinearHeading(RUNG_BLUE_AUDIENCE, TANGENT_225_DEGREES)
                .splineToLinearHeading(OBSERVATION_BLUE_ZONE, TANGENT_225_DEGREES)
                .build();

        /** RED AUDIENCE **/
        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RUNG_RED_BACKSTAGE, TANGENT_225_DEGREES)
                .splineToLinearHeading(OBSERVATION_RED_ZONE, TANGENT_225_DEGREES)
                .splineToLinearHeading(RUNG_RED_BACKSTAGE, TANGENT_225_DEGREES)
                .splineToLinearHeading(OBSERVATION_RED_ZONE, TANGENT_225_DEGREES)
                .splineToLinearHeading(RUNG_RED_BACKSTAGE, TANGENT_225_DEGREES)
                .splineToLinearHeading(OBSERVATION_RED_ZONE, TANGENT_225_DEGREES)
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


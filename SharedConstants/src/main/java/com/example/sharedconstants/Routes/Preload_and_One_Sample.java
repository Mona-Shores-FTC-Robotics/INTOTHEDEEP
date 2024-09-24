package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_ZONE_RED_PICKUP;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HOME_POSITION;
import static com.example.sharedconstants.RobotAdapter.ActionType.SECURE_PRELOAD_SPECIMEN;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.example.sharedconstants.RobotAdapter;

public class Preload_and_One_Sample extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public static Action blueTestRoute;

    public Preload_and_One_Sample(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        redBackstageBotRoute = robotAdapter.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_RED_BACKSTAGE))
                .build();

        blueAudienceBotRoute = robotAdapter.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_RED_BACKSTAGE))
                .build();

        redAudienceBotRoute = robotAdapter.actionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();

        blueBackstageBotRoute = robotAdapter.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();
    }

    public class RouteBuilder {
        Action ScorePreloadSpecimen(Pose2d startPose, Pose2d chamberPose) {
            return new SequentialAction(
                 robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN),
                 new ParallelAction(
                    robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER),
                    DriveToChamber(startPose, chamberPose)
                 ),
                 robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER)

            );
        }

        Action DriveToChamber(Pose2d startPose, Pose2d chamberPose) {
            return robotAdapter.actionBuilder(startPose)
                    .splineToLinearHeading(chamberPose, chamberPose.heading, slowVelocity, slowAcceleration)
                    .build();
        }

        Action DriveToObservationZone(Pose2d startPose, Pose2d observationZonePose) {
            return new ParallelAction(
                    robotAdapter.actionBuilder(startPose)
                            .setReversed(true)
                            .splineToLinearHeading(observationZonePose, observationZonePose.heading.inverse(), slowVelocity, slowAcceleration)
                            .build(),
                    robotAdapter.getAction(LIFT_TO_HOME_POSITION)
            );
        }
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

package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.example.sharedconstants.RobotAdapter;
import static com.example.sharedconstants.RobotAdapter.ActionType.*;

public class PreloadRoute extends Routes {

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public static Action blueTestRoute;

    public PreloadRoute(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        redBackstageBotRoute = robotAdapter.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_RED_BACKSTAGE))
//                .stopAndAdd(new RouteBuilder().DriveToObservationZone(CHAMBER_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
                .build();

        blueAudienceBotRoute = robotAdapter.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_RED_BACKSTAGE))
//                .stopAndAdd(new RouteBuilder().DriveToObservationZone(CHAMBER_RED_BACKSTAGE, OBSERVATION_ZONE_RED_PICKUP))
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
                    //Secure the preloaded specimen
                    robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN),

                    //Drive to the chamber while raising the lift
                    new ParallelAction(
                            new RouteBuilder().DriveToChamber(startPose, chamberPose),
                            robotAdapter.getAction(LIFT_TO_LOW_CHAMBER)
                            ),

                    //Help us to go slow and see what the robot is doing
                    new SleepAction(1),

                    //Hang the specimen on the chamber
                    robotAdapter.getAction(HANG_SPECIMEN_ON_LOW_CHAMBER)
            );
        }

        Action DriveToChamber(Pose2d startPose, Pose2d chamberPose) {
            return robotAdapter.actionBuilder(startPose)
                    .splineToLinearHeading(chamberPose, chamberPose.heading, slowVelocity, slowAcceleration)
                    .build();
        }

        Action DriveToObservationZone(Pose2d startPose, Pose2d observationZonePose) {
            return robotAdapter.actionBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(observationZonePose, observationZonePose.heading.inverse(), slowVelocity, slowAcceleration)
                    .build();
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
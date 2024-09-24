package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HOME_POSITION;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL;
import static com.example.sharedconstants.RobotAdapter.ActionType.SECURE_PRELOAD_SPECIMEN;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public abstract class Routes {

    // Velocity and acceleration overrides
    public static final double SLOW_VELOCITY_OVERRIDE = 10;
    public static final double SLOW_ACCELERATION_OVERRIDE = 15;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(45);

    public static final double NORMAL_VELOCITY_OVERRIDE = 30;
    public static final double NORMAL_ACCELERATION_OVERRIDE = 35;
    public static final double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static final double FAST_VELOCITY_OVERRIDE = 40;
    public static final double FAST_ACCELERATION_OVERRIDE = 45;
    public static final double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(120);

    // Shared constraints for all routes
    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint normalVelocity;
    public static AccelConstraint normalAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;

    protected RobotAdapter robotAdapter;

    public Routes(RobotAdapter robotAdapter) {
        this.robotAdapter = robotAdapter;
        setupConstraints();
    }

    public abstract void BuildRoutes();

    // Variables to store routes for all start locations and team prop locations
    protected Action redAudienceBotRoute;
    protected Action redBackstageBotRoute;
    protected Action blueBackstageBotRoute;
    protected Action blueAudienceBotRoute;

    // These are the defaults
    //THIS SAYS RED AUDIENCE BECXAUSE IT IS MIRRORED
    protected Pose2d redAudienceStartPose = RED_AUDIENCE_START_POSE;
    protected Pose2d blueBackstageStartPose = BLUE_BACKSTAGE_START_POSE;

    //THIS SAYS RED BACKSTAGE BECAUSE ITS MIRRORED
    protected Pose2d redBackstageStartPose = RED_BACKSTAGE_START_POSE;
    protected Pose2d blueAudienceStartPose = BLUE_AUDIENCE_START_POSE;

    // Method to return the corresponding starting pose
    public Pose2d getStartingPose(AllianceColor allianceColor, SideOfField sideOfField) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        return blueAudienceStartPose;
                    case BACKSTAGE:
                        return blueBackstageStartPose;
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        return redAudienceStartPose;
                    case BACKSTAGE:
                        return redBackstageStartPose;
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    public Action getRoute(AllianceColor allianceColor, SideOfField sideOfField) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        return getBlueAudienceBotRoute();  // Consolidated method for BLUE AUDIENCE
                    case BACKSTAGE:
                        return getBlueBackstageBotRoute();  // Consolidated method for BLUE BACKSTAGE
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        return getRedAudienceBotRoute();  // Consolidated method for RED AUDIENCE
                    case BACKSTAGE:
                        return getRedBackstageBotRoute();  // Consolidated method for RED BACKSTAGE
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
    }

    public Action getBlueAudienceBotRoute() {
        return blueAudienceBotRoute;
    }

    public Action getBlueBackstageBotRoute() {
        return blueBackstageBotRoute;
    }

    public Action getRedBackstageBotRoute() {
        return redBackstageBotRoute;
    }

    private void setupConstraints() {
        slowVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SLOW_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)
        ));
        slowAcceleration = new ProfileAccelConstraint(-SLOW_ACCELERATION_OVERRIDE, SLOW_ACCELERATION_OVERRIDE);

        normalVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(NORMAL_VELOCITY_OVERRIDE),
                new AngularVelConstraint(NORMAL_ANGULAR_VELOCITY_OVERRIDE)
        ));
        normalAcceleration = new ProfileAccelConstraint(-NORMAL_ACCELERATION_OVERRIDE, NORMAL_ACCELERATION_OVERRIDE);

        fastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)
        ));
        fastAcceleration = new ProfileAccelConstraint(-FAST_ACCELERATION_OVERRIDE, FAST_ACCELERATION_OVERRIDE);
    }

    public class RouteBuilder {

        Action ScorePreloadSpecimen(Pose2d startPose, Pose2d chamberPose) {
            return new SequentialAction(
                    robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN),
                    new ParallelAction(
                            robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER),
                            DriveToChamberFromStart(startPose, chamberPose)
                    ),
                    robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER)
            );
        }

        Action ScoreSpecimen(Pose2d startPose, Pose2d chamberPose) {
            return new SequentialAction(
                    robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN),
                    new ParallelAction(
                            robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER),
                            DriveToChamberFromObservation(startPose, chamberPose)
                    ),
                    robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER)
            );
        }

        Action PickupSpecimen(Pose2d startPose, Pose2d observationZonePose) {
            return new SequentialAction(
                        new ParallelAction(
                            DriveToObservationZone(startPose, observationZonePose),
                            robotAdapter.getAction(LIFT_TO_HOME_POSITION)
                        ),
                        robotAdapter.getAction(PICKUP_SPECIMEN_OFF_WALL)
            );
        }
        Action DriveToChamberFromStart(Pose2d startPose, Pose2d chamberPose) {
            return robotAdapter.actionBuilder(startPose)
                    .splineToLinearHeading(chamberPose, chamberPose.heading, slowVelocity, slowAcceleration)
                    .build();
        }

        Action DriveToChamberFromObservation(Pose2d observationZonePose, Pose2d chamberPose) {
            return robotAdapter.actionBuilder(observationZonePose)
                    .setReversed(true)
                    .splineToLinearHeading(chamberPose, chamberPose.heading, slowVelocity, slowAcceleration)
                    .build();
        }


        Action DriveToObservationZone(Pose2d startPose, Pose2d observationZonePose) {
            return robotAdapter.actionBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(observationZonePose, observationZonePose.heading, slowVelocity, slowAcceleration)
                    .build();
        }
    }
}
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
import com.acmerobotics.roadrunner.NullAction;
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
    public static final double SLOW_VELOCITY_OVERRIDE = 20;
    public static final double SLOW_ACCELERATION_OVERRIDE = 20;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

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
    //THIS SAYS RED AUDIENCE BECAUSE IT IS MIRRORED - DO NOT CHANGE THESE
    protected Pose2d redAudienceStartPose = RED_AUDIENCE_START_POSE;
    protected Pose2d blueBackstageStartPose = RED_AUDIENCE_START_POSE;

    //THIS SAYS RED BACKSTAGE BECAUSE ITS MIRRORED - DO NOT CHANGE THESE
    protected Pose2d redBackstageStartPose = RED_BACKSTAGE_START_POSE;
    protected Pose2d blueAudienceStartPose = RED_BACKSTAGE_START_POSE;

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
                    //TODO: write this code for scoring a preloaded specimen
                        // Make sure the preloaded specimen is secure
                        // Drive to the Chamber from the Start Position while moving the lift
                        // Hang the specimen on the high chamber
                   robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN),
                   new ParallelAction(
                          DriveToChamberFromStart(startPose,chamberPose),
                          robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER)
                    ),
                    robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER)
            );
        }

        Action ScoreSpecimen(Pose2d startPose, Pose2d chamberPose) {
            return new SequentialAction(
                    //TODO: write this code for scoring a specimen
                        // Drive to the Chamber from the Observation Zone while moving the lift in parallel
                        // Hang the specimen on the high chamber
                   new ParallelAction(
                           DriveToChamberFromObservation(startPose,chamberPose),
                           robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER)
                   ),
                    robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER)
            );
        }

        Action PickupSpecimen(Pose2d startPose, Pose2d observationZonePose) {
            return new SequentialAction(
                    //TODO: write this code for picking up a specimen
                        //Drive to the Observation Zone while lowering the lift in parallel
                        //Pickup the specimen off the wall
                    new ParallelAction(
                            DriveToObservationZoneFromChamber(startPose,observationZonePose),
                            robotAdapter.getAction(LIFT_TO_HOME_POSITION)
                    ),
                    robotAdapter.getAction(PICKUP_SPECIMEN_OFF_WALL),
                    robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN)
            );
        }
        Action DriveToChamberFromStart(Pose2d startPose, Pose2d chamberPose) {
            //TODO: write this code for driving to the chamber  from the start position
            return
                    robotAdapter.actionBuilder(startPose)
                            .splineToLinearHeading(chamberPose,chamberPose.heading,slowVelocity,slowAcceleration)
                            .build();
        }

        Action DriveToChamberFromObservation(Pose2d observationZonePose, Pose2d chamberPose) {
            //TODO: write this code for driving to the chamber from the observation zone
            return
                    robotAdapter.actionBuilder(observationZonePose)
                            .setReversed(true)
                            .splineToLinearHeading(chamberPose,chamberPose.heading,slowVelocity,slowAcceleration)
                        .build();
        }

        Action DriveToObservationZoneFromChamber(Pose2d chamberPose, Pose2d observationZonePose) {
            //TODO: write this code for driving to the chamber from the observation zone
            return robotAdapter.actionBuilder(chamberPose)
                    .setReversed(true)
                    .splineToLinearHeading(observationZonePose,observationZonePose.heading,slowVelocity,slowAcceleration)
                    .build();
        }

        public Action NullDriveAction(Pose2d currentPose) {
            // Small move to stabilize position visualization in MeepMeep
            return robotAdapter.actionBuilder(currentPose)
                    .splineToLinearHeading(
                            new Pose2d(currentPose.position.x + 0.000001,
                                    currentPose.position.y,
                                    currentPose.heading.log()),
                            currentPose.heading)
                    .build();
        }
    }
}
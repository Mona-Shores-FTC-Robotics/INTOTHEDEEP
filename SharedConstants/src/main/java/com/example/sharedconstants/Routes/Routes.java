package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;
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
    public static final double SLOW_VELOCITY_OVERRIDE = 20;
    public static final double SLOW_ACCELERATION_OVERRIDE = 20;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static final double NORMAL_VELOCITY_OVERRIDE = 30;
    public static final double NORMAL_ACCELERATION_OVERRIDE = 35;
    public static final double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    public static final double FAST_VELOCITY_OVERRIDE = 40;
    public static final double FAST_ACCELERATION_OVERRIDE = 45;
    public static final double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

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
    protected Action netBotRoute = null;
    protected Action observationBotRoute = null;

    public Action getRouteAction(SideOfField sideOfField) {
        if (sideOfField == SideOfField.NET) {
            return getNetBotRoute();  // Consolidated method for BLUE AUDIENCE
        } else return getObservationBotRoute();
    }

    // Get the NetBotRoute only if it's available
    public Action getNetBotRoute() {
        if (netBotRoute == null) {
            System.out.println("Net bot route not available.");
            return null;
        }
        return netBotRoute;
    }

    // Get the ObservationBotRoute only if it's available
    public Action getObservationBotRoute() {
        if (observationBotRoute == null) {
            // Optional: Log or notify that the route is not available
            System.out.println("Observation bot route not available.");
            return null;
        }
        return observationBotRoute;
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

        Action PickupSpecimen(Pose2d startPose, Pose2d waypoint, Pose2d observationZonePose) {
            return new SequentialAction(
                    //TODO: write this code for picking up a specimen
                        //Drive to the Observation Zone while lowering the lift in parallel
                        //Pickup the specimen off the wall
                    new ParallelAction(
                            DriveToObservationZoneFromChamber(startPose, waypoint, observationZonePose),
                            robotAdapter.getAction(HOME)
                    ),
                    robotAdapter.getAction(PICKUP_SPECIMEN_OFF_WALL),
                    robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN)
            );
        }
        Action DriveToChamberFromStart(Pose2d startPose, Pose2d chamberPose) {
            //TODO: write this code for driving to the chamber  from the start position
            return
                    robotAdapter.getActionBuilder(startPose)
                            .splineToLinearHeading(chamberPose,chamberPose.heading,slowVelocity,slowAcceleration)
                            .build();
        }

        Action DriveToNeutralFromChamber(Pose2d chamberPose, Pose2d netNeutralSpikeOnePose) {
            //TODO: write this code for driving to the neutral spike one from the chamber position
            return
                    robotAdapter.getActionBuilder(chamberPose)
                            .splineToLinearHeading(netNeutralSpikeOnePose,netNeutralSpikeOnePose.heading,slowVelocity,slowAcceleration)
                            .build();
        }

        Action DriveToNetFromSpike(Pose2d netNeutralSpikeOnePose, Pose2d netZoneScore) {
            //TODO: write this code for driving to the neutral spike one from the chamber position
            return
                    robotAdapter.getActionBuilder(netNeutralSpikeOnePose)
                            .splineToLinearHeading(netZoneScore,netZoneScore.heading,slowVelocity,slowAcceleration)
                            .build();
        }
        Action DriveToChamberFromObservation(Pose2d observationZonePose, Pose2d chamberPose) {
            //TODO: write this code for driving to the chamber from the observation zone
            return
                    robotAdapter.getActionBuilder(observationZonePose)
                            .setReversed(true)
                            .splineToLinearHeading(chamberPose,chamberPose.heading,slowVelocity,slowAcceleration)
                        .build();
        }

        Action DriveToObservationZoneFromChamber(Pose2d chamberPose,Pose2d waypoint, Pose2d observationZonePose) {
            //TODO: write this code for driving to the chamber from the observation zone
            return robotAdapter.getActionBuilder(chamberPose)
                    .setReversed(true)
                    .splineToLinearHeading(waypoint,waypoint.heading,slowVelocity,slowAcceleration)
                    .splineToLinearHeading(observationZonePose,observationZonePose.heading,slowVelocity,slowAcceleration)
                    .build();
        }

        public Action NullDriveAction(Pose2d currentPose) {
            // Small move to stabilize position visualization in MeepMeep
            return robotAdapter.getActionBuilder(currentPose)
                    .splineToLinearHeading(
                            new Pose2d(currentPose.position.x + 0.000001,
                                    currentPose.position.y,
                                    currentPose.heading.log()),
                            currentPose.heading)
                    .build();
        }
    }
}
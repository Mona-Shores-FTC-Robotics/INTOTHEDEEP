package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SECURE_PRELOAD_SPECIMEN;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public abstract class Routes {

    // Velocity and acceleration overrides
    public static final double SLOW_VELOCITY_OVERRIDE = 20;
    public static final double SLOW_ACCELERATION_OVERRIDE = 20;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static final double NORMAL_VELOCITY_OVERRIDE = 25;
    public static final double NORMAL_ACCELERATION_OVERRIDE = 25;
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
    protected TrajectoryActionBuilder obsTrajectoryActionBuilder;
    protected TrajectoryActionBuilder netTrajectoryActionBuilder;

    // Variables to store routes for all start locations and team prop locations
    protected Action netBotRoute = null;
    protected Action observationBotRoute = null;

    public Routes(RobotAdapter robotAdapter) {
        this.robotAdapter = robotAdapter;
        setupConstraints();
    }

    public TrajectoryActionBuilder getObsTrajectoryActionBuilder() {
        return obsTrajectoryActionBuilder;
    }

    public TrajectoryActionBuilder getNetTrajectoryActionBuilder() {
        return  netTrajectoryActionBuilder;
    }

    public Action getRouteAction(SideOfField sideOfField) {
        if (sideOfField == SideOfField.NET) {
            return getNetBotRoute();
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

    public void buildRoute(){

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

    public void pickupSpecimenFromWall(Boolean reversed) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(reversed)
                .splineToLinearHeading(OBS_ZONE_PICKUP, ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL));
    }

    public void scoreOnHighChamber(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToSplineHeading(CHAMBER_STAGING, ANGLE_135_DEGREES)
                .splineToLinearHeading(chamberSlot, ANGLE_135_DEGREES)
                .afterDisp(.1, robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void pushFirstNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(Math.toRadians(-10))
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_ONE), ANGLE_TOWARD_OBSERVATION)
                .splineToLinearHeading(OBS_WAYPOINT, ANGLE_TOWARD_OBSERVATION);
    }

    public void pushSecondNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToLinearHeading(OBS_BEHIND_SPIKE_TWO, ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_DELIVER_SPIKE_TWO, ANGLE_TOWARD_RED);
    }

    public void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(Math.toRadians(-10))
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(NEXT_TO_OBS_ASCENT), ANGLE_TOWARD_BLUE)
                .splineToLinearHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_DELIVER_SPIKE_THREE, ANGLE_TOWARD_RED);
    }
}
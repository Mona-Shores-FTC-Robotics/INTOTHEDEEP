package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

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
    public static final double SLOW_VELOCITY_OVERRIDE = 5;
    public static final double SLOW_ACCELERATION_OVERRIDE = 5;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    public static final double NORMAL_VELOCITY_OVERRIDE = 23;
    public static final double NORMAL_ACCELERATION_OVERRIDE = 20;
    public static final double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    public static final double FAST_VELOCITY_OVERRIDE = 30;
    public static final double FAST_ACCELERATION_OVERRIDE = 30;
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
}
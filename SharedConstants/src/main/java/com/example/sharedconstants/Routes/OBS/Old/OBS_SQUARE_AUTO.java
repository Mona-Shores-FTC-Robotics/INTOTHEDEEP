package com.example.sharedconstants.Routes.OBS.Old;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.AUTO_TEST_POSE;
import static com.example.sharedconstants.FieldConstants.TILE;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;


@AutoRoute
public class OBS_SQUARE_AUTO extends Routes {

    public OBS_SQUARE_AUTO(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    private static final double PRELOAD_VELOCITY_OVERRIDE = 24;
    private static final double PRELOAD_ACCELERATION_OVERRIDE = 24;
    private static final double PRELOAD_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double PRELOAD_SLOW_VELOCITY_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ACCELERATION_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);


    public void buildRoute() {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(AUTO_TEST_POSE);
        driveForwardToBlueObs();
        turnRight();
        driveForwardToBlueNet();
        turnRight();
        driveForwardToRedObs();
        turnRight();
        driveForwardToRedNet();
        turnRight();
        driveForwardToBlueObs();
        turnRight();
        driveForwardToBlueNet();
        turnRight();
        driveForwardToRedObs();
        turnRight();
        driveForwardToRedNet();

        observationBotRoute= obsTrajectoryActionBuilder.build();
    }

    private void turnRight() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .turn(Math.toRadians(-90));
    }

    private void driveForwardToBlueObs() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(new Vector2d(AUTO_TEST_POSE.position.x, AUTO_TEST_POSE.position.y+4*TILE), ANGLE_TOWARD_BLUE);
    }
    private void driveForwardToBlueNet() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(new Vector2d(AUTO_TEST_POSE.position.x+4*TILE, AUTO_TEST_POSE.position.y+4*TILE), ANGLE_TOWARD_OBSERVATION);
    }
    private void driveForwardToRedObs() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(new Vector2d(AUTO_TEST_POSE.position.x+4*TILE, AUTO_TEST_POSE.position.y), ANGLE_TOWARD_RED);
    }
    private void driveForwardToRedNet() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(new Vector2d(AUTO_TEST_POSE.position.x, AUTO_TEST_POSE.position.y), ANGLE_TOWARD_NET);
    }


}

package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ELEVEN;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TEN;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWELVE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.MOVE_PRELOAD_SPECIMEN_TO_CW_HOME;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;

import jdk.internal.misc.CDS;

public class OBS_Score_1_Specimen_Preload extends Routes {

    public OBS_Score_1_Specimen_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        //todo it looks to me like we are not matching the pose correctly in the x axis with the slot - some testing is needed here with our parameters.
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    //todo what if we tested putting a high velocity but putting limits on acceleration. See https://rr.brott.dev/docs/v1-0/guides/variable-constraints/
    private static final double PRELOAD_VELOCITY_OVERRIDE = 27;
    private static final double PRELOAD_ACCELERATION_OVERRIDE = 27;
    private static final double PRELOAD_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(600);

    private static final double PRELOAD_SLOW_VELOCITY_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ACCELERATION_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public void scoreObservationPreload(Pose2d chamberSlot) {

        VelConstraint preloadVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(PRELOAD_VELOCITY_OVERRIDE),
                new AngularVelConstraint(PRELOAD_ANGULAR_VELOCITY_OVERRIDE)
        ));
        AccelConstraint preloadAcceleration = new ProfileAccelConstraint(- PRELOAD_ACCELERATION_OVERRIDE, PRELOAD_ACCELERATION_OVERRIDE);

        VelConstraint preloadSlowVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(PRELOAD_SLOW_VELOCITY_OVERRIDE),
                new AngularVelConstraint(PRELOAD_SLOW_ANGULAR_VELOCITY_OVERRIDE)
        ));
        AccelConstraint preloadSlowAcceleration = new ProfileAccelConstraint(- PRELOAD_SLOW_ACCELERATION_OVERRIDE, PRELOAD_SLOW_ACCELERATION_OVERRIDE);


        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToSplineHeading(chamberSlot.plus(new Twist2d(new Vector2d(-10,0), 0)), chamberSlot.heading.toDouble(), preloadVelocity, preloadAcceleration)
                .splineToSplineHeading(chamberSlot, chamberSlot.heading.toDouble(), preloadSlowVelocity, preloadSlowAcceleration)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.2);
    }
}

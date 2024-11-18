package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.*;
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
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;


@AutoRoute
public class NET_Score_1_Specimen_Preload extends Routes {

    public NET_Score_1_Specimen_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    private static final double PRELOAD_VELOCITY_OVERRIDE = 24;
    private static final double PRELOAD_ACCELERATION_OVERRIDE = 24;
    private static final double PRELOAD_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double PRELOAD_SLOW_VELOCITY_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ACCELERATION_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);


    public void buildRoute() {


        scoreNetSpecimenPreload(CHAMBER_SLOT_SEVEN_ROTATED);
        netBotRoute= netTrajectoryActionBuilder.build();
    }

    public void scoreNetSpecimenPreload(Pose2d chamberSlot){
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

        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToSplineHeading(chamberSlot.plus(new Twist2d(new Vector2d(-8,0), 0)), chamberSlot.heading.toDouble(), preloadVelocity, preloadAcceleration)
                .splineToSplineHeading(chamberSlot, chamberSlot.heading.toDouble(), preloadSlowVelocity, preloadSlowAcceleration)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.2);



    }
}

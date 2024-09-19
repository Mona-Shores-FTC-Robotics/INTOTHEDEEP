package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotDriveAdapter;

import java.util.Arrays;

public class PreloadRoute extends Routes {

    public static double SLOW_VELOCITY_OVERRIDE = 10;
    public static double SLOW_ACCELERATION_OVERRIDE = 15;
    public static double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(45);

    public static double FAST_VELOCITY_OVERRIDE = 40;
    public static double FAST_ACCELERATION_OVERRIDE = 40;
    public static double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static double SUPER_FAST_VELOCITY_OVERRIDE = 40;
    public static double SUPER_FAST_ACCELERATION_OVERRIDE = 40;
    public static double SUPER_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;
    public static VelConstraint superFastVelocity;
    public static AccelConstraint superFastAcceleration;

    public Action redAudienceBotRoute;
    public Action redBackstageBotRoute;
    public Action blueAudienceBotRoute;
    public Action blueBackstageBotRoute;

    public static Action blueTestRoute;

    public PreloadRoute(RobotDriveAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        SetupConstraints();

        redBackstageBotRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_RED_BACKSTAGE))
                .build();

        redAudienceBotRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();

        blueAudienceBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_BACKSTAGE_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();

        blueBackstageBotRoute = roadRunnerDrive.mirroredActionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RouteBuilder().ScorePreloadSpecimen(RED_AUDIENCE_START_POSE, CHAMBER_RED_AUDIENCE))
                .build();
    }

    private void SetupConstraints() {
        VelConstraint baseVelConstraint =
                slowVelocity = new MinVelConstraint(Arrays.asList(
                        new TranslationalVelConstraint(SLOW_VELOCITY_OVERRIDE),
                        new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)
                ));

        slowAcceleration = new ProfileAccelConstraint(-SLOW_ACCELERATION_OVERRIDE, SLOW_ACCELERATION_OVERRIDE);

        fastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)));
        fastAcceleration = new ProfileAccelConstraint(-FAST_ACCELERATION_OVERRIDE, FAST_ACCELERATION_OVERRIDE);

        superFastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SUPER_FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SUPER_FAST_ANGULAR_VELOCITY_OVERRIDE)));
        superFastAcceleration = new ProfileAccelConstraint(-SUPER_FAST_ACCELERATION_OVERRIDE, SUPER_FAST_ACCELERATION_OVERRIDE);

    }

    public class RouteBuilder {

        Action ScorePreloadSpecimen(Pose2d startPose, Pose2d chamberPose) {
            SequentialAction scorePreloadSpecimen =
                    new SequentialAction(
                            roadRunnerDrive.createCloseGripperAction(),
                            new ParallelAction(
                                    new RouteBuilder().DriveToChamber(startPose, chamberPose),
                                    new SequentialAction(
                                            new SleepAction(.6),
                                            new ActuateEndEffectorAction(GripperStates.OPEN)
                                    )
                            )
                    );
            return scorePreloadSpecimen;
        }

        Action DriveToChamber(Pose2d startPose, Pose2d chamberPose) {
            Action driveToChamber = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(chamberPose, chamberPose.heading, slowVelocity, slowAcceleration)
                    .build();
            return driveToChamber;
        }


    }
}

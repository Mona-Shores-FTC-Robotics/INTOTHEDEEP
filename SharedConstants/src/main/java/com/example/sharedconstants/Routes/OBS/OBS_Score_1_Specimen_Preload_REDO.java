package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.HALF_TILE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.QUARTER_TILE;
import static com.example.sharedconstants.FieldConstants.THREE_QUARTER_TILE;
import static com.example.sharedconstants.FieldConstants.TILE;
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

public class OBS_Score_1_Specimen_Preload_REDO extends Routes {

    public OBS_Score_1_Specimen_Preload_REDO(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        SetupConstraints();
        scoreObservationPreload(CHAMBER_SLOT_ONE_REDO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    private static final double OBS_VELOCITY_OVERRIDE = 55;
    private static final double OBS_SLOW_VELOCITY_OVERRIDE = 22;
    private static final double OBS_VERY_SLOW_VELOCITY_OVERRIDE = 10;

    public static VelConstraint obsVelocity;

    public void SetupConstraints() {
        obsVelocity = (robotPose , _path , _disp) -> {
            // Extract X and Y values for readability
            double x = robotPose.position.x.value();
            double y = robotPose.position.y.value();

            //Check if we are close to the chamber and should slow down
            if (x > - HALF_TILE && x < HALF_TILE) {
                //Check Red Side Chamber
                if (y > - TILE - HALF_TILE && y < - HALF_TILE ||
                        //Check Blue Side Chamber
                        (y < TILE + HALF_TILE && y > HALF_TILE)) {
                    return OBS_SLOW_VELOCITY_OVERRIDE; // Override constraint
                }

                //Check if near Red Observation Zone
            } else if (y < - 2 * TILE - QUARTER_TILE-.5 && x > TILE+HALF_TILE-2 ||
                    //Check if near Blue Observation Zone
                    (y > 2 * TILE + QUARTER_TILE+.5 && x < - TILE-HALF_TILE+2)) {
                return OBS_VERY_SLOW_VELOCITY_OVERRIDE; // Override constraint
            }
            return OBS_VELOCITY_OVERRIDE; // Default constraint
        };
    }


    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToSplineHeading(chamberSlot.plus(new Twist2d(new Vector2d(-10,0), 0)), chamberSlot.heading.toDouble(), obsVelocity)
                .splineToConstantHeading(PoseToVector(chamberSlot), chamberSlot.heading.toDouble(), obsVelocity)
                //todo this sequence might be able to be sped up
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.2);
    }
}

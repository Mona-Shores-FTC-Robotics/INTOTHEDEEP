package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.SPECIMEN_ARM_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SECURE_PRELOAD_SPECIMEN;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class OBS_Score_1_Specimen_Preload extends Routes {

    public OBS_Score_1_Specimen_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(OBS_START_POSE)
                .splineToLinearHeading(chamberSlot, CHAMBER_SLOT_ONE.heading.toDouble())
                .afterDisp(0, robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN))
                .afterDisp(4, robotAdapter.getAction(SPECIMEN_ARM_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(3, -3)), ANGLE_TOWARD_OBSERVATION)
                .afterDisp(3, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME));
    }

}

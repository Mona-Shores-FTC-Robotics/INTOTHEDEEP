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

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

import jdk.internal.misc.CDS;

public class OBS_Score_1_Specimen_Preload extends Routes {

    public OBS_Score_1_Specimen_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {

        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToLinearHeading(chamberSlot, chamberSlot.heading.toDouble())
                .waitSeconds(.1)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)));
    }
}

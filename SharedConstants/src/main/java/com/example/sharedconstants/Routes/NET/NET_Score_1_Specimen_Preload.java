package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.SPECIMEN_ARM_TO_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;


@AutoRoute
public class NET_Score_1_Specimen_Preload extends Routes {

    public NET_Score_1_Specimen_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        scorePreloadOnSeventhChamberSlot();
        netBotRoute= netTrajectoryActionBuilder.build();
    }

    public void scorePreloadOnSeventhChamberSlot(){
        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE)
                .splineToLinearHeading(CHAMBER_SLOT_SEVEN, CHAMBER_SLOT_SEVEN.heading.toDouble())
                .afterDisp(.1,  robotAdapter.getAction(SPECIMEN_ARM_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .strafeTo(PoseToVector(CHAMBER_SLOT_SEVEN).minus(new Vector2d(0,3)))
                .afterDisp(0, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME));
    }
}

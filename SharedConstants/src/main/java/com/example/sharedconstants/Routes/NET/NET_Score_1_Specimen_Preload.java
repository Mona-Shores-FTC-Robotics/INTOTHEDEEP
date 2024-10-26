package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.*;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SECURE_PRELOAD_SPECIMEN;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.RouteRegistry;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.RoutesToRun;


@AutoRoute
public class NET_Score_1_Specimen_Preload extends Routes {
    // Static block for self-registration
    static {
        RouteRegistry.registerRoute(RoutesToRun.NET_SCORE_1_PRELOAD, NET_Score_1_Specimen_Preload::new);
    }

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
                .afterDisp(0,   robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN))
                .afterDisp(.1,  robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .strafeTo(PoseToVector(CHAMBER_SLOT_SEVEN).minus(new Vector2d(0,3)))
                .afterDisp(0, robotAdapter.getAction(HOME));
    }
}

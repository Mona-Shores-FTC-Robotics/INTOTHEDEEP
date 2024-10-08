package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

// General-purpose adapter for robot actions
public interface RobotAdapter {

    // Enum for actions that both RealRobot and MeepMeep should handle
    enum ActionType {
        SECURE_PRELOAD_SPECIMEN,
        PICKUP_SPECIMEN_OFF_WALL,
        HANG_SPECIMEN_ON_HIGH_CHAMBER,
        HANG_SPECIMEN_ON_LOW_CHAMBER,
        PICKUP_SAMPLE,
        DEPOSIT_SAMPLE,
        LIFT_TO_HIGH_CHAMBER,
        LIFT_TO_LOW_CHAMBER,
        HOME,
        LIFT_TO_HIGH_BASKET,
        LIFT_TO_LOW_BASKET,
        LEVEL_1_ASCENT,
        LEVEL_2_ASCENT,
        LEVEL_3_ASCENT
    }

    // Generalized method to get an Action based on a provided action type
    Action getAction(ActionType actionType);

    // The new method that will determine which builder to use
    TrajectoryActionBuilder getActionBuilder(Pose2d startPose);

    void setAllianceColor(FieldConstants.AllianceColor allianceColor);
    void setSideOfField(FieldConstants.SideOfField sideOfField);

}
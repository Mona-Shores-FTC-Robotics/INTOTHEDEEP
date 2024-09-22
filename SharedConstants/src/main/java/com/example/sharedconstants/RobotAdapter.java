package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

// General-purpose adapter for robot actions
public interface RobotAdapter {

    // Enum for actions that both RealRobot and MeepMeep should handle
    enum ActionType {
        SECURE_PRELOAD_SPECIMEN,
        PICKUP_SPECIMEN,
        HANG_SPECIMEN_ON_HIGH_CHAMBER,
        HANG_SPECIMEN_ON_LOW_CHAMBER,
        PICKUP_SAMPLE,
        DEPOSIT_SAMPLE,
        LIFT_TO_HIGH_CHAMBER,
        LIFT_TO_LOW_CHAMBER,
        LIFT_TO_HOME_POSITION,
        LIFT_TO_HIGH_BASKET,
        LIFT_TO_LOW_BASKET
    }

    // Generalized method to get an Action based on a provided action type
    Action getAction(ActionType actionType);

    // Method for creating trajectory actions
    TrajectoryActionBuilder actionBuilder(Pose2d startPose);

    // Method for mirrored driving actions
    TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose);
}
package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

// General-purpose adapter for robot actions
public interface RobotAdapter {

    // Enum for actions that both RealRobot and MeepMeep should handle
    enum ActionType {
        SAMPLE_INTAKE_ON,
        SAMPLE_INTAKE_OFF,
        SAMPLE_INTAKE_REVERSE,

        SPECIMEN_INTAKE_ON,
        SPECIMEN_INTAKE_OFF,
        SPECIMEN_INTAKE_REVERSE,

        SECURE_PRELOAD_SPECIMEN,
        PICKUP_SPECIMEN_OFF_WALL,

        SPECIMEN_ARM_TO_HIGH_CHAMBER,
        HANG_SPECIMEN_ON_HIGH_CHAMBER,

        INTAKE_SAMPLE_FROM_GROUND,
        SAMPLE_LIFT_TO_HOME,
        SAMPLE_LIFT_TO_HIGH_BASKET,
        SAMPLE_LIFT_TO_LOW_BASKET,
        DEPOSIT_SAMPLE,

        DUMP_SAMPLE_IN_OBSERVATION_ZONE,

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
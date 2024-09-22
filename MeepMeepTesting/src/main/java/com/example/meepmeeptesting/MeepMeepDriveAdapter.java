package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotAdapter;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class MeepMeepDriveAdapter implements RobotAdapter {
    private final DriveShim driveShim;
    private final ActionFactory actionFactory;

    public MeepMeepDriveAdapter(DriveShim driveShim) {
        this.driveShim = driveShim;
        this.actionFactory = new ActionFactory();  // Initialize the ActionFactory
    }

    @Override
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return driveShim.actionBuilder(startPose);
    }

    @Override
    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d startPose) {
        return driveShim.actionBuilder(startPose);
    }

    @Override
    public Action getAction(ActionType actionType) {
        return actionFactory.createAction((ActionType) actionType);
    }

    // Inner ActionFactory class
    private static class ActionFactory {
        public Action createAction(ActionType actionType) {
            switch (actionType) {
                case SECURE_PRELOAD_SPECIMEN:
                    return new SleepAction(.5);  // Simulating a close gripper action
                case PICKUP_SPECIMEN:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case HANG_SPECIMEN_ON_HIGH_CHAMBER:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case HANG_SPECIMEN_ON_LOW_CHAMBER:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case PICKUP_SAMPLE:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case DEPOSIT_SAMPLE:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case LIFT_TO_HIGH_CHAMBER:
                    return new SleepAction(1);  // Simulating lift to high chamber
                case LIFT_TO_LOW_CHAMBER:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case LIFT_TO_HOME_POSITION:
                    return new SleepAction(1);  // Simulating lift to home position
                case LIFT_TO_HIGH_BASKET:
                    return new SleepAction(.5);  // Simulating a close gripper action

                case LIFT_TO_LOW_BASKET:
                    return new SleepAction(.5);  // Simulating a close gripper action

                default:
                    return new SleepAction(1);
            }
        }
    }
}

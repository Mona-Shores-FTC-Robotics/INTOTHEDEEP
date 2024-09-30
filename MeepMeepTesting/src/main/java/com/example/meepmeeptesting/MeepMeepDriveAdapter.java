package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotAdapter;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import org.jetbrains.annotations.NotNull;

public class MeepMeepDriveAdapter implements RobotAdapter {
    private final DriveShim driveShim;
    private final ActionFactory actionFactory;

    public MeepMeepDriveAdapter(DriveShim driveShim) {
        this.driveShim = driveShim;
        this.actionFactory = new ActionFactory(driveShim);  // Initialize the ActionFactory
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
        return actionFactory.createAction(actionType);
    }

    // Inner ActionFactory class
    private static class ActionFactory {
        private final DriveShim driveShim;

        public ActionFactory(DriveShim driveShim) {
            this.driveShim = driveShim;  // Store the driveShim instance
        }

        public Action createAction(ActionType actionType) {
            switch (actionType) {
                default:
                    return new SleepAction(1);
            }
        }
    }

    public Pose2d getCurrentPose(){
        return driveShim.getPoseEstimate();
    }

}

package com.example.meepmeeptesting;

import static com.example.sharedconstants.FieldConstants.CHAMBER_RED_BACKSTAGE_VEC;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotDriveAdapter;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.acmerobotics.roadrunner.Pose2d;


public class MeepMeepDriveAdapter implements RobotDriveAdapter {
    private final DriveShim driveShim;

    public MeepMeepDriveAdapter(DriveShim driveShim) {
        this.driveShim = driveShim;
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
    public Action createCloseGripperAction() {
        return new SleepAction(.5);
    }

    @Override
    public Action createOpenGripperAction() {
        return new SleepAction(.5);
    }

    @Override
    public Action createLiftToHighChamberAction() {
        return new SleepAction(1);
    }

    @Override
    public Action createLiftToHomePosistionAction() {
        return new SleepAction(1);
    }


}
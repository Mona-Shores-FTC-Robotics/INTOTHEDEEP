package com.example.meepmeeptesting;

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
    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        return driveShim.actionBuilder(beginPose);
    }
}
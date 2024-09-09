package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.example.sharedconstants.RobotDriveAdapter;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.roadrunner.entity.TurnAction;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryAction;

import java.util.Arrays;


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
        return null;
    }
}
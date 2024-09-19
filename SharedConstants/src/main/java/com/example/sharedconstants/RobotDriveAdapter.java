package com.example.sharedconstants;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public interface RobotDriveAdapter {
    TrajectoryActionBuilder actionBuilder(Pose2d startPose);
    TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose);

    Action createCloseGripperAction();
    Action createOpenGripperAction();
}


package com.example.sharedconstants;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public interface RobotDriveAdapter {
    TrajectoryActionBuilder actionBuilder(Pose2d startPose);
    TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose);

    Action createCloseGripperAction();
    Action createOpenGripperAction();
    Action createLiftToHighChamberAction();
    Action createLiftToHomePosistionAction();

    //what are all the actions we are going to have to call in auto?

}


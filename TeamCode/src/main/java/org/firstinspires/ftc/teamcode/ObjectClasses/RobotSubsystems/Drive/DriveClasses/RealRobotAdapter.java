package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.example.sharedconstants.RobotDriveAdapter;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive.TurnAction;
import org.firstinspires.ftc.teamcode.MecanumDrive.FollowTrajectoryAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class RealRobotAdapter implements RobotDriveAdapter {
    private final MecanumDriveMona drive;

    public RealRobotAdapter(MecanumDriveMona drive) {
        this.drive = drive;
    }

    @Override
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }

    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        return drive.mirroredActionBuilder(beginPose);
    }
}

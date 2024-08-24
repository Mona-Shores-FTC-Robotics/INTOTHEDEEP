package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotDriveAdapter;

public class RealRobotAdapter implements RobotDriveAdapter {
    private final MecanumDriveMona drive;

    public RealRobotAdapter(MecanumDriveMona drive) {
        this.drive = drive;
    }

    @Override
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }
}

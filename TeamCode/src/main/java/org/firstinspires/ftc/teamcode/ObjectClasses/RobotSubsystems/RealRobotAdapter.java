package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotDriveAdapter;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses.MecanumDriveMona;

public class RealRobotAdapter implements RobotDriveAdapter {
    private final MecanumDriveMona drive;
    private final GripperSubsystem gripperSubsystem;

    public RealRobotAdapter() {
        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();
    }

    @Override
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }

    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        return drive.mirroredActionBuilder(beginPose);
    }

    @Override
    public Action createCloseGripperAction() {
        return null;
    }

    @Override
    public Action createOpenGripperAction() {
        return null;
    }
}

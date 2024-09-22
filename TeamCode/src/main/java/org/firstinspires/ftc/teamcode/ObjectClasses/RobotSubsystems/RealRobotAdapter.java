package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotDriveAdapter;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses.MecanumDriveMona;

public class RealRobotAdapter implements RobotDriveAdapter {
    private final MecanumDriveMona drive;
    private final GripperSubsystem gripperSubsystem;
    private final LiftSlideSubsystem liftSubystem;

    public RealRobotAdapter() {
        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        liftSubystem = Robot.getInstance().getLiftSlideSubsystem();
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
        return new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED);
    }

    @Override
    public Action createOpenGripperAction() {
        return new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN);
    }

    @Override
    public Action createLiftToHighChamberAction() {
        return new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.HIGH);
    }

    @Override
    public Action createLiftToHomePosistionAction() {
        return null;
    }
}

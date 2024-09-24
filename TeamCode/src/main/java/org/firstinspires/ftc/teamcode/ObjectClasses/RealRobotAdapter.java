package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotAdapter;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftAction;

public class RealRobotAdapter implements RobotAdapter {
    private final ActionFactory actionFactory;

    public RealRobotAdapter() {
        actionFactory = new ActionFactory();
    }

    @Override
    public Action getAction(ActionType actionType) {
        return actionFactory.createAction(actionType);
    }

    @Override
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(startPose);
    }

    @Override
    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        return Robot.getInstance().getDriveSubsystem().mecanumDrive.mirroredActionBuilder(beginPose);
    }

    // Inner ActionFactory class
    private static class ActionFactory {
        public ActionFactory() {
        }

        public Action createAction(ActionType actionType) {
            switch (actionType) {
                case SECURE_PRELOAD_SPECIMEN:
                case PICKUP_SPECIMEN:
                case PICKUP_SAMPLE:
                    return new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED);
                case HANG_SPECIMEN_ON_HIGH_CHAMBER:
                    return new SequentialAction(
                            new MoveLiftAction(LiftSubsystem.LiftStates.HANG_HIGH_CHAMBER),
                            new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN));
                case HANG_SPECIMEN_ON_LOW_CHAMBER:
                    return new SequentialAction(
                            new MoveLiftAction(LiftSubsystem.LiftStates.HANG_LOW_CHAMBER),
                            new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN));
                case DEPOSIT_SAMPLE:
                    return new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN);
                case LIFT_TO_HIGH_CHAMBER:
                    return new MoveLiftAction(LiftSubsystem.LiftStates.HIGH_CHAMBER);
                case LIFT_TO_LOW_CHAMBER:
                    return new MoveLiftAction(LiftSubsystem.LiftStates.LOW_CHAMBER);
                case LIFT_TO_HIGH_BASKET:
                    return new MoveLiftAction(LiftSubsystem.LiftStates.HIGH_BASKET);
                case LIFT_TO_LOW_BASKET:
                    return new MoveLiftAction(LiftSubsystem.LiftStates.LOW_BASKET);
                case LIFT_TO_HOME_POSITION:
                    return new MoveLiftAction(LiftSubsystem.LiftStates.HOME);
                default:
                    throw new IllegalArgumentException("Unknown action type: " + actionType);
            }
        }
    }
}

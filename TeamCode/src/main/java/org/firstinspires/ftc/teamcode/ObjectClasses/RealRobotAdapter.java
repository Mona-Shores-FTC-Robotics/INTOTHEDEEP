package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        public Action createAction(ActionType actionType) {
            Robot robot = Robot.getInstance();

            switch (actionType) {
                case SECURE_PRELOAD_SPECIMEN:
                case PICKUP_SPECIMEN_OFF_WALL:
                case PICKUP_SAMPLE:
                    if (robot.hasSubsystem(Robot.SubsystemType.GRIPPER)) {
                        return new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED);
                    } else {
                        robot.getActiveOpMode().telemetry.addLine("Gripper subsystem is not available on this robot.");
                        robot.getActiveOpMode().telemetry.update();
                        return new SleepAction(0.1);  // Returning nonce action
                    }

                case HANG_SPECIMEN_ON_HIGH_CHAMBER:
                    if (robot.hasSubsystem(Robot.SubsystemType.LIFT) && robot.hasSubsystem(Robot.SubsystemType.GRIPPER)) {
                        return new SequentialAction(
                                new MoveLiftAction(LiftSubsystem.LiftStates.HANG_HIGH_CHAMBER),
                                new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN));
                    } else {
                        robot.getActiveOpMode().telemetry.addLine("Lift or Gripper subsystem is not available on this robot.");
                        robot.getActiveOpMode().telemetry.update();
                        return new SleepAction(0.1);  // Returning nonce action
                    }

                case HANG_SPECIMEN_ON_LOW_CHAMBER:
                    if (robot.hasSubsystem(Robot.SubsystemType.LIFT) && robot.hasSubsystem(Robot.SubsystemType.GRIPPER)) {
                        return new SequentialAction(
                                new MoveLiftAction(LiftSubsystem.LiftStates.HANG_LOW_CHAMBER),
                                new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN));
                    } else {
                        robot.getActiveOpMode().telemetry.addLine("Lift or Gripper subsystem is not available on this robot.");
                        robot.getActiveOpMode().telemetry.update();
                        return new SleepAction(0.1);  // Returning nonce action
                    }

                case DEPOSIT_SAMPLE:
                    if (robot.hasSubsystem(Robot.SubsystemType.GRIPPER)) {
                        return new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN);
                    } else {
                        robot.getActiveOpMode().telemetry.addLine("Gripper subsystem is not available on this robot.");
                        robot.getActiveOpMode().telemetry.update();
                        return new SleepAction(0.1);  // Returning nonce action
                    }

                case LIFT_TO_HIGH_CHAMBER:
                case LIFT_TO_LOW_CHAMBER:
                case LIFT_TO_HIGH_BASKET:
                case LIFT_TO_LOW_BASKET:
                case LIFT_TO_HOME_POSITION:
                    if (robot.hasSubsystem(Robot.SubsystemType.LIFT)) {
                        return new MoveLiftAction(LiftSubsystem.LiftStates.valueOf(actionType.name()));
                    } else {
                        robot.getActiveOpMode().telemetry.addLine("Lift subsystem is not available on this robot.");
                        robot.getActiveOpMode().telemetry.update();
                        return new SleepAction(0.1);  // Returning nonce action
                    }

                default:
                    robot.getActiveOpMode().telemetry.addLine("Unknown action type: " + actionType);
                    robot.getActiveOpMode().telemetry.update();
                    return new SleepAction(0.1);  // Returning nonce action for unknown actions
            }
        }
    }
}
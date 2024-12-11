package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryActionStub;
import com.noahbres.meepmeep.roadrunner.entity.TurnActionStub;

import org.jetbrains.annotations.NotNull;

public class MeepMeepRobotAdapter implements RobotAdapter {
    private final DriveShim driveShim;
    private final ActionFactory actionFactory;
    private TrajectoryActionBuilder trajectoryActionBuilder;  // Store a single builder instance
    private FieldConstants.AllianceColor allianceColor;
    private FieldConstants.SideOfField sideOfField;

    public MeepMeepRobotAdapter(DriveShim driveShim) {
        this.driveShim = driveShim;
        this.actionFactory = new ActionFactory(driveShim);  // Initialize the ActionFactory
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return driveShim.actionBuilder(startPose);
    }

    public TrajectoryActionBuilder rotatedActionBuilder(Pose2d beginPose) {
        // Return a TrajectoryActionBuilder using the mirrored pose and constraints
        return new TrajectoryActionBuilder(
                TurnActionStub::new,
                TrajectoryActionStub::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(.25, .1, 1e-2)),
                beginPose,
                0.0,
                driveShim.actionBuilder(beginPose).getBaseTurnConstraints(),   // Apply the turn constraints
                driveShim.actionBuilder(beginPose).getBaseVelConstraint(), // Apply the velocity constraint
                driveShim.actionBuilder(beginPose).getBaseAccelConstraint(), // Apply the acceleration constraint
                pose ->
                        new Pose2dDual<>(
                        pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.plus(Math.toRadians(180))));
    }

    @Override
    public Action getAction(ActionType actionType) {
        return actionFactory.createAction(actionType);
    }

    // Inner ActionFactory class
    private static class ActionFactory {
        private final DriveShim driveShim;

        public ActionFactory(DriveShim driveShim) {
            this.driveShim = driveShim;  // Store the driveShim instance
        }

        public Action createAction(ActionType actionType) {
            switch (actionType) {
                case CONDITIONAL_PICKUP:
                    return new SleepAction(.8);
                case CONDITIONAL_TRANSFER:
                    return new SleepAction(.3);

                default:
                    return new SleepAction(.2);
            }
        }
    }

    public void setAllianceColor(FieldConstants.AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void setSideOfField(FieldConstants.SideOfField sideOfField) {
        this.sideOfField = sideOfField;
    }

    public TrajectoryActionBuilder getActionBuilder(Pose2d startPose) {
            if (isRotated()) {
                return rotatedActionBuilder(startPose);
            } else {
                return actionBuilder(startPose);
            }
    }

    public boolean isRotated() {
        return this.allianceColor == FieldConstants.AllianceColor.BLUE; // Return true if the alliance color is BLUE
    }
}

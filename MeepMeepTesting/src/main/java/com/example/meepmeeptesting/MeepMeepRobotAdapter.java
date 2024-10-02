package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
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
        // Define constraints similar to the original DriveShim
        TurnConstraints turnConstraints = new TurnConstraints(
                Math.toRadians(180),  // maxAngVel
                Math.toRadians(90),   // minAngAccel
                Math.toRadians(180)   // maxAngAccel
        );

        VelConstraint baseVelConstraint = (robotPose, path, s) -> {
            return 40;  // Hardcoded max velocity
        };

        AccelConstraint baseAccelConstraint = (robotPose, path, s) -> {
            double minAccel = -30;  // Example min acceleration (negative for deceleration)
            double maxAccel = 30;   // Example max acceleration

            return new MinMax(minAccel, maxAccel);
        };

        // Return a TrajectoryActionBuilder using the mirrored pose and constraints
        return new TrajectoryActionBuilder(
                TurnActionStub::new,
                TrajectoryActionStub::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(.25, .1, 1e-2)),
                beginPose,
                0.0,
                turnConstraints,   // Apply the turn constraints
                baseVelConstraint, // Apply the velocity constraint
                baseAccelConstraint, // Apply the acceleration constraint
                pose -> new Pose2dDual<>(
                        pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.inverse()));
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
                default:
                    return new SleepAction(1);
            }
        }
    }

    public Pose2d getCurrentPose(){
        return driveShim.getPoseEstimate();
    }

    @Override
    public void setAllianceColor(FieldConstants.AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }
    @Override
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

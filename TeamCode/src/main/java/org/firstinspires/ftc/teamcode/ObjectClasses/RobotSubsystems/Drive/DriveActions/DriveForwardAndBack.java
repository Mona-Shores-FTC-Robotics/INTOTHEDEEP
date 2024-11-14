package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class DriveForwardAndBack implements Action {
    private final DriveSubsystem driveSubsystem;
    private final RealRobotAdapter robotAdapter;
    private final double distance;
    private Action driveForwardAndBack;
    private boolean initialized = false;

    public DriveForwardAndBack(double distance) {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.robotAdapter = new RealRobotAdapter();
        this.distance = distance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            Pose2d initialPose = driveSubsystem.getMecanumDrive().pose;
            Vector2d forwardPosition = calculateTargetPosition(initialPose, distance);
            Vector2d backwardPosition = new Vector2d(initialPose.position.x, initialPose.position.y);

            // Set up the forward and backward actions as one combined action
            driveForwardAndBack = robotAdapter.actionBuilder(initialPose)
                    .strafeTo(forwardPosition)
                    .strafeTo(backwardPosition)
                    .waitSeconds(.1)
                    .build();
            initialized = true;
        }

        // Execute the combined action
        boolean actionComplete = driveForwardAndBack.run(telemetryPacket);

        // Update telemetry for diagnostics
        telemetryPacket.put("DriveForwardAndBack", actionComplete ? "Complete" : "In Progress");

        // Return true if the action has completed its forward and back movement
        return actionComplete;
    }

    private Vector2d calculateTargetPosition(Pose2d referencePose, double movementDistance) {
        double heading = referencePose.heading.log();
        double offsetX = movementDistance * Math.cos(heading);
        double offsetY = movementDistance * Math.sin(heading);
        return new Vector2d(referencePose.position.x + offsetX, referencePose.position.y + offsetY);
    }

    private void reset() {
        initialized = false;
    }
}

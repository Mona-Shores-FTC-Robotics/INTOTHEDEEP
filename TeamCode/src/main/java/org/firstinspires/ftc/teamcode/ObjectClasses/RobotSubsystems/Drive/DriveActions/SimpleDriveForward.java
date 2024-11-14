package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class SimpleDriveForward implements Action {
    private final DriveSubsystem driveSubsystem;
    private final RealRobotAdapter robotAdapter;
    private final double distance = 10; // Drive forward 10 inches
    private Action driveAction;
    private boolean initialized = false;

    public SimpleDriveForward() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.robotAdapter = new RealRobotAdapter();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            // Get initial pose
            Pose2d initialPose = driveSubsystem.getMecanumDrive().pose;
            // Calculate target position based on initial pose and distance
            Vector2d forwardPosition = calculateTargetPosition(initialPose, distance);

            // Set up the forward movement action
            driveAction = robotAdapter.actionBuilder(initialPose)
                    .splineToConstantHeading(forwardPosition, initialPose.heading.log())
                    .build();

            initialized = true;
        }

        // Run the forward movement action
        boolean isComplete = driveAction.run(telemetryPacket);
        telemetryPacket.put("SimpleDriveForward", isComplete ? "Complete" : "In Progress");

        return isComplete;
    }

    private Vector2d calculateTargetPosition(Pose2d referencePose, double movementDistance) {
        double heading = referencePose.heading.log();
        double offsetX = movementDistance * Math.cos(heading);
        double offsetY = movementDistance * Math.sin(heading);
        return new Vector2d(referencePose.position.x + offsetX, referencePose.position.y + offsetY);
    }
}

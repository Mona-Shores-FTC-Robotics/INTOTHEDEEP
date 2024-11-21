package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.PoseToVector;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class SimpleDriveForward implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private Action action;
    private double distance;

    public SimpleDriveForward(double distance) {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.distance=distance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!started) {
            RealRobotAdapter robotAdapter = new RealRobotAdapter();
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;

            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                currentPose = new Pose2d(-currentPose.position.x, -currentPose.position.y, currentPose.heading.log()+PI);
            }

            double heading = currentPose.heading.log();
            double offsetX = distance * Math.cos(heading);
            double offsetY = distance * Math.sin(heading);

            Vector2d targetVector = new Vector2d(
                    currentPose.position.x + offsetX,
                    currentPose.position.y + offsetY);

            action = robotAdapter.getActionBuilder(currentPose)
                    .setReversed(true)
                    .splineToConstantHeading(targetVector, Math.toRadians(180)+currentPose.heading.log()).build();

            action.preview(MatchConfig.telemetryPacket.fieldOverlay()); // Optional: Preview for telemetry
            started = true; // Ensure the action is only initialized once
        }

        // Run the action and check if it's still running
        boolean isRunning = action.run(telemetryPacket);

        if (!isRunning) {
            // Reset the state when the action completes
            reset();
        }

        // Update telemetry with current pose information
        telemetryPacket.put("x", driveSubsystem.getMecanumDrive().pose.position.x);
        telemetryPacket.put("y", driveSubsystem.getMecanumDrive().pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(driveSubsystem.getMecanumDrive().pose.heading.log()));
        telemetryPacket.put("SimpleDriveForward", isRunning ? "In Progress" : "Complete");

        return isRunning;
    }

    // Method to reset the state of the action
    private void reset() {
        started = false;
        action = null;
    }
}

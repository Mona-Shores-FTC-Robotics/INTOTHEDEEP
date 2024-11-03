package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class DriveForwardAction implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private boolean cancelled;
    private Action action;// Flag to indicate if the action has been cancelled

    public DriveForwardAction() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false; // Initialize the cancellation flag
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            // Stop the robot abruptly when cancelled
            driveSubsystem.getMecanumDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            reset(); // Reset the state to allow re-running the action
            return false; // Signal that the action is complete
        }

        if (!started) {
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;
            action = driveSubsystem.getMecanumDrive()
                    .actionBuilder(currentPose)
                    .setReversed(false)
                    .splineToConstantHeading(
                            new Vector2d(currentPose.position.x, currentPose.position.y + -10.0),
                            currentPose.heading.log()
                    )
                    .build();

            action.preview(MatchConfig.telemetryPacket.fieldOverlay()); // Optional: Preview for telemetry
            started = true; // Ensure the action is only initialized once
        }

        // Run the action and update telemetry
        boolean isRunning = action.run(telemetryPacket);

        if (!isRunning) {
            reset(); // Reset the state when the action completes
        }

        telemetryPacket.put("x", driveSubsystem.getMecanumDrive().pose.position.x);
        telemetryPacket.put("y", driveSubsystem.getMecanumDrive().pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(driveSubsystem.getMecanumDrive().pose.heading.log()));
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        return isRunning;
    }

    public void cancelAbruptly() {
        cancelled = true; // Set the cancellation flag
    }

    // Method to reset the state of the action
    private void reset() {
        started = false;
        cancelled = false;
        action = null; // Clear the action for potential re-initialization
    }
}

package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP_FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
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
            return false; // Signal that the action is complete
        }

        if (!started) {
            RealRobotAdapter robotAdapter = new RealRobotAdapter();
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;
            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                currentPose = new Pose2d(-currentPose.position.x, -currentPose.position.y, currentPose.heading.log()+PI);
            }

            action = robotAdapter.getActionBuilder(currentPose)
                    .setReversed(true)
                    .splineToLinearHeading(FieldConstants.OBS_ZONE_PICKUP, ANGLE_TOWARD_RED).build();

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
        return isRunning;
    }

    public void cancelAbruptly() {
        cancelled = true; // Set the cancellation flag
    }

    // Method to reset the state of the action
    private void reset() {
        started = false;
        cancelled = false;
        action = null; // Reset the action reference
    }
}

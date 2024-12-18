package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
@Config
public class BetterPrepareAction implements Action {

    public static double DELAY_TIME_TO_DRIVE_FORWARD=1;

    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private boolean cancelled;
    private Action action;// Flag to indicate if the action has been cancelled
    private final double distance;

    // Shared constraints for all routes
    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;

    public BetterPrepareAction(double distance) {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.distance = distance;

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

            double heading = currentPose.heading.log();
            double offsetX = distance * Math.cos(heading);
            double offsetY = distance * Math.sin(heading);

            Vector2d targetVector = new Vector2d(
                    currentPose.position.x + offsetX,
                    currentPose.position.y + offsetY);

            action = robotAdapter.getActionBuilder(currentPose)
                    .stopAndAdd(new  SequentialAction(
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveLiftToHighBasket),
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveDumperToPreScore),
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::setBucketToScorePosition)))
                    .waitSeconds(DELAY_TIME_TO_DRIVE_FORWARD)
                    .setReversed(true)
                    .splineToConstantHeading(targetVector, Math.toRadians(180)+currentPose.heading.log()).build();

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

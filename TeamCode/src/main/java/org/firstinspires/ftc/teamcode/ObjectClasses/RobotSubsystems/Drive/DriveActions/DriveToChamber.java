package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class DriveToChamber implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private boolean cancelled;
    private Action action;
    private int slotNumber = 0; // Track the current slot number

    public DriveToChamber() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false;
    }

    public DriveToChamber(double inches) {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            driveSubsystem.getMecanumDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return false;
        }

        if (!started) {
            RealRobotAdapter robotAdapter = new RealRobotAdapter();
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;

            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                currentPose = new Pose2d(-currentPose.position.x, -currentPose.position.y, currentPose.heading.log() + PI);
            }

            action = robotAdapter.getActionBuilder(currentPose)
                    .setReversed(false)
                    .splineToLinearHeading(FieldConstants.CHAMBER_SLOT_THREE, ANGLE_TOWARD_RED)
                    .build();

            action.preview(MatchConfig.telemetryPacket.fieldOverlay());
            started = true;
        }

        boolean isRunning = action.run(telemetryPacket);

        if (!isRunning) {
            reset();
        }

        telemetryPacket.put("x", driveSubsystem.getMecanumDrive().pose.position.x);
        telemetryPacket.put("y", driveSubsystem.getMecanumDrive().pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(driveSubsystem.getMecanumDrive().pose.heading.log()));
        telemetryPacket.put("slotNumber", slotNumber); // Display slot number in telemetry
        return isRunning;
    }

    public void cancelAbruptly() {
        cancelled = true;
    }

    private void reset() {
        started = false;
        cancelled = false;
        action = null;
        slotNumber++; // Increment slot number each time the action completes
    }
}

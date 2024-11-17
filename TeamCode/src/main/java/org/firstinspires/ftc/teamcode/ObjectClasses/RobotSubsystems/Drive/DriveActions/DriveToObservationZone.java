package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

import java.util.Arrays;
import java.util.Set;

public class DriveToObservationZone implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private boolean cancelled;
    private Action action;// Flag to indicate if the action has been cancelled

    // Velocity and acceleration overrides
    public static final double VELOCITY_OVERRIDE = 45;
    public static final double ACCELERATION_OVERRIDE = 45;
    public static final double ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    // Shared constraints for all routes
    public static VelConstraint velConstraint;
    public static AccelConstraint accelConstraint;

    public DriveToObservationZone() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false; // Initialize the cancellation flag
    }

    public DriveToObservationZone(double inches) {
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

            velConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(VELOCITY_OVERRIDE),
                    new AngularVelConstraint(ANGULAR_VELOCITY_OVERRIDE)
            ));
            accelConstraint = new ProfileAccelConstraint(- ACCELERATION_OVERRIDE, ACCELERATION_OVERRIDE);

            RealRobotAdapter robotAdapter = new RealRobotAdapter();
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;

            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                currentPose = new Pose2d(-currentPose.position.x, -currentPose.position.y, currentPose.heading.log()+PI);
            }

            action = robotAdapter.getActionBuilder(currentPose)
                    .setTangent(ANGLE_TOWARD_OBSERVATION)
                    .splineToLinearHeading(FieldConstants.OBS_TRIANGLE_APPROACH, ANGLE_TOWARD_RED)
                    .setReversed(true)
                    .afterDisp(0, new InstantAction(Robot.getInstance().getSpecimenArmSubsystem()::gotoPickupAngle))
                    .afterDisp(0, new InstantAction(Robot.getInstance().getSpecimenIntakeSubsystem()::turnOnIntake))
                    .splineToLinearHeading(FieldConstants.OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED)
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

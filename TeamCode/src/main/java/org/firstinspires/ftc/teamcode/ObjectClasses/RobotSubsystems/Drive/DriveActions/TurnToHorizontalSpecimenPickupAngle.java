package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class TurnToHorizontalSpecimenPickupAngle implements Action {
    private final DriveSubsystem driveSubsystem;
    private final RealRobotAdapter robotAdapter;
    private Action turnToBucketAngle;
    private boolean initialized = false;

    public TurnToHorizontalSpecimenPickupAngle() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.robotAdapter = new RealRobotAdapter();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            Pose2d initialPose = driveSubsystem.getMecanumDrive().pose;

            // Set up the forward and backward actions as one combined action
            turnToBucketAngle = robotAdapter.actionBuilder(initialPose)
                    .turnTo(Math.toRadians(MatchConfig.finalAllianceColor == BLUE ? 180 : 0))
                    .build();
            initialized = true;
        }

        // Execute the combined action
        boolean actionComplete = turnToBucketAngle.run(telemetryPacket);

        // Update telemetry for diagnostics
        telemetryPacket.put("TurnToBucketAngle", actionComplete ? "Complete" : "In Progress");

        // Return true if the action has completed its forward and back movement
        return actionComplete;
    }
}

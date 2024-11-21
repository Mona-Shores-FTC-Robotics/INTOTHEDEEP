package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import static com.example.sharedconstants.FieldConstants.*;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToObservationZone;

public class MatchConfig {

    public static AllianceColor finalAllianceColor = AllianceColor.RED;
    public static AllianceColor finalOpponentColor = AllianceColor.BLUE;
    public static SideOfField finalSideOfField = SideOfField.OBSERVATION;
    public static Robot.RobotType finalRobotType = Robot.RobotType.INTO_THE_DEEP_20245;  // This should be set when the robot is created

    public static double offsetFromStartPoseDegrees;

    public static Pose2d endOfAutonomousPose = null;

    public static ElapsedTime teleOpTimer;
    public static ElapsedTime loopTimer;
    public static ElapsedTime timestampTimer;

    public static TelemetryPacket telemetryPacket;

    // Sliding window average variables
    private static final int WINDOW_SIZE = 100;
    private static final double[] loopTimes = new double[WINDOW_SIZE];
    public static boolean hasAutoRun=false;
    private static int loopIndex = 0;
    private static int loopCount = 0; // Number of recorded loop times
    private static double totalLoopTime = 0;

    public static void addLoopTime(double loopTime) {
        // If the window is full, subtract the value being overwritten
        if (loopCount == WINDOW_SIZE) {
            totalLoopTime -= loopTimes[loopIndex];
        } else {
            loopCount++;
        }

        // Add the new loop time and update the total
        loopTimes[loopIndex] = loopTime;
        totalLoopTime += loopTime;

        // Move to the next index in the array (wrap around if needed)
        loopIndex = (loopIndex + 1) % WINDOW_SIZE;
    }

    public static double getAverageLoopTime() {
        return loopCount > 0 ? totalLoopTime / loopCount : 0.0;
    }


}

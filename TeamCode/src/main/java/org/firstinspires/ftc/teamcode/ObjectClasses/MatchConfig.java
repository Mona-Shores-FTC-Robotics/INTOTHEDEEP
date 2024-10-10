package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import static com.example.sharedconstants.FieldConstants.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MatchConfig {

    public static AllianceColor finalAllianceColor = AllianceColor.BLUE;
    public static SideOfField finalSideOfField = SideOfField.OBSERVATION;
    public static Robot.RobotType finalRobotType = Robot.RobotType.CHASSIS_19429_B_PINPOINT;  // Default to an initial type

    public static boolean verboseMode = false;

    public static Pose2d endOfAutonomousPose = null;
    public static double endOfAutonomousOffset;
    public static double endOfAutonomousAbsoluteYawDegrees;

    public static ElapsedTime teleOpTimer;
    public static ElapsedTime loopTimer;
    public static ElapsedTime timestampTimer;

    public static TelemetryPacket telemetryPacket;

    // Sliding window average variables
    private static final int WINDOW_SIZE = 100;
    private static final double[] loopTimes = new double[WINDOW_SIZE];
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

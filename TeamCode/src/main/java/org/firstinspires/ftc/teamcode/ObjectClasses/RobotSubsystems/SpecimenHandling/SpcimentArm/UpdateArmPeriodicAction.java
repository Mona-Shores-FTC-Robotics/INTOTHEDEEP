package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class UpdateArmPeriodicAction implements Action {
    private final SpecimenArmWithMotionProfileSubsystem armSubsystem;

    public UpdateArmPeriodicAction(SpecimenArmWithMotionProfileSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Add the loop time to the sliding window average
        MatchConfig.addLoopTime( MatchConfig.loopTimer.milliseconds());

        // Reset the loop timer
        MatchConfig.loopTimer.reset();

        armSubsystem.periodic();  // Call the periodic method to update the arm

        // Display Telemetry through the Robot's Telemetry Manager
        Robot.getInstance().getDriverStationTelemetryManager().displayTelemetry();
        // Send packet to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(MatchConfig.telemetryPacket);
        // Clear the packet for the next loop
        MatchConfig.telemetryPacket = new TelemetryPacket();
        return true;
    }
}
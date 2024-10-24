package org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class RotateShoulderAction implements Action {
    private ShoulderSubsystem.ShoulderStates targetState;

    public RotateShoulderAction(ShoulderSubsystem.ShoulderStates inputState) {
            targetState = inputState;
        }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
        telemetryPacket.put("Target Shoulder State", targetState);
        Robot.getInstance().getShoulderSubsystem().shoulder.setPosition(targetState.position);
        Robot.getInstance().getShoulderSubsystem().setCurrentState(targetState);
        return false;
    }
}

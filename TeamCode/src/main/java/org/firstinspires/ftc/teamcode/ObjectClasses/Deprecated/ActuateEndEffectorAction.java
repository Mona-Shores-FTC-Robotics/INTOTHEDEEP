package org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class ActuateEndEffectorAction implements Action {

    //declare target state & position
    private GripperSubsystem.GripperStates targetState;
    private double targetPosition;

    public ActuateEndEffectorAction(GripperSubsystem.GripperStates inputState) {
        //save the input state, s, as the target state
        targetState = inputState;
        //get the target position from the input state
        targetPosition = targetState.position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getEndEffectorSubsystem().endEffector.setPosition(targetPosition);
        telemetryPacket.put("Current EndEffector State", Robot.getInstance().getEndEffectorSubsystem().currentState);
        telemetryPacket.put("Target EndEffector State: ", targetState);
        Robot.getInstance().getEndEffectorSubsystem().currentState = targetState;
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        return false;
    }
}


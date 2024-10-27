package org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public final class GripperSubsystem extends SubsystemBase {

    public static GripperStates GRIPPER_STARTING_STATE = GripperStates.OPEN;
    public static double OPEN_POSITION = .45;
    public static double CLOSED_POSITION = .55;

    public enum GripperStates {
        CLOSED (.55),
        OPEN (.45);
        public double position;
        GripperStates(double pos) {
            this.position = pos;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public final Servo endEffector;
    public GripperStates currentState;

    public GripperSubsystem(final HardwareMap hMap, final String name) {
        endEffector = hMap.get(Servo.class, name);
    }

    public void init() {
        currentState= GRIPPER_STARTING_STATE;
        endEffector.setPosition(currentState.position);
    }

    public void periodic(){
        GripperStates.CLOSED.SetState(CLOSED_POSITION);
        GripperStates.OPEN.SetState(OPEN_POSITION);

        //Add the Gripper State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Gripper State", currentState);
    }

}

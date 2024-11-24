package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;

public final class SampleLinearActuatorMessage {
    public long timestamp;

    public SampleLinearActuatorSubsystem.SampleActuatorStates currentState;

    public SampleLinearActuatorMessage(SampleLinearActuatorSubsystem.SampleActuatorStates currentState) {
        this.timestamp = System.nanoTime();
        this.currentState = currentState;
    }
}

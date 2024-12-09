package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleTwister;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ConfigurableParameters;

@Config
public class SampleTwisterSubsystem extends SubsystemBase {

    public static class SampleTwisterParams extends ConfigurableParameters {
        public double TWISTER_FACE_DUMP;
        public double TWISTER_FACE_OUTWARDS;
        public double TWISTER_FACE_INWARDS;

        @Override
        public void loadDefaultsForRobotType(Robot.RobotType robotType) {
            if (haveRobotSpecificParametersBeenLoaded()) return; // Skip reloading if already customized

            switch (robotType) {
                case INTO_THE_DEEP_19429:
                    TWISTER_FACE_INWARDS = 1.0;
                    TWISTER_FACE_OUTWARDS = .6;
                    TWISTER_FACE_DUMP = .31;
                    break;
                case INTO_THE_DEEP_20245:
                    TWISTER_FACE_INWARDS = 0.63;
                    TWISTER_FACE_OUTWARDS = 0.23;
                    TWISTER_FACE_DUMP = .8;
                    break;

                default:
                    throw new IllegalArgumentException("Unknown robot type: " + robotType);
            }
            markRobotSpecificParametersLoaded(); // Flag parameters as customized
        }
    }

    public static SampleTwisterParams SAMPLE_TWISTER_PARAMS = new SampleTwisterParams();

    public enum SampleTwisterStates {
        FACE_INWARDS,
        FACE_OUTWARDS, FACE_DUMP_SIDE;

        public double getTwisterPosition() {
            switch (this) {
                case FACE_INWARDS:
                    return SAMPLE_TWISTER_PARAMS.TWISTER_FACE_INWARDS;
                case FACE_OUTWARDS:
                    return SAMPLE_TWISTER_PARAMS.TWISTER_FACE_OUTWARDS;
                case FACE_DUMP_SIDE:
                    return SAMPLE_TWISTER_PARAMS.TWISTER_FACE_DUMP;
                default:
                    throw new IllegalStateException("Power not defined for state: " + this);
            }
        }
    }
    private final Servo sampleTwister;  // Continuous rotation servo
    private SampleTwisterStates currentSampleTwisterState;
    private double currentPosition;

    // Constructor with color sensor
    public SampleTwisterSubsystem(final HardwareMap hMap, Robot.RobotType robotType, final String twisterServoName) {
        SAMPLE_TWISTER_PARAMS.loadDefaultsForRobotType(robotType);
        sampleTwister = hMap.get(Servo.class, twisterServoName);
    }

    // Initialize intake servo
    public void init() {
        setTwisterServoFaceInward();
    }

    @Override
    public void periodic() {

    }

    // Set servo power, ensuring it's within limits
    private void setTwisterState(SampleTwisterStates state) {
        sampleTwister.setPosition(state.getTwisterPosition());
    }

    // Set the current intake state and update power
    public void setTwisterServoFaceInward() {
        SampleTwisterStates intakeOffState = SampleTwisterStates.FACE_INWARDS;
        sampleTwister.setPosition(intakeOffState.getTwisterPosition());
    }

    // Set the current intake state and update power
    public void setTwisterServoFaceOutwards() {
        SampleTwisterStates intakeOffState = SampleTwisterStates.FACE_OUTWARDS;
        sampleTwister.setPosition(intakeOffState.getTwisterPosition());
    }
    // Set the current intake state and update power
    public void setTwisterServoDumpToSide() {
        SampleTwisterStates intakeOffState = SampleTwisterStates.FACE_DUMP_SIDE;
        sampleTwister.setPosition(intakeOffState.getTwisterPosition());
    }
}

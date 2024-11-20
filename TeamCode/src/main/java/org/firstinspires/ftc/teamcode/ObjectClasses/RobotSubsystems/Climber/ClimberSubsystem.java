package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class ClimberSubsystem extends SubsystemBase {

    public static void configureParamsForRobotType(Robot.RobotType robotType) {
        switch (robotType) {
            case INTO_THE_DEEP_19429:
                CLIMBER_PARAMS.CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED;
                CLIMBER_PARAMS.CLIMBER_MOTOR_STARTING_STATE = ClimberMotorStates.OFF;

                CLIMBER_PARAMS.READY_POSITION = 0;
                CLIMBER_PARAMS.STOWED_STEP1_VALUE = .6;
                CLIMBER_PARAMS.STOWED_STEP2_VALUE = .53;
                CLIMBER_PARAMS.STOWED_STEP3_VALUE = .5;
                CLIMBER_PARAMS.STOWED_POSITION = 1;

                CLIMBER_PARAMS.ROBOT_DOWN_POWER = -0.8;
                CLIMBER_PARAMS.ROBOT_UP_POWER = 0.8;
                break;

            case INTO_THE_DEEP_20245:
                CLIMBER_PARAMS.CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED;
                CLIMBER_PARAMS.CLIMBER_MOTOR_STARTING_STATE = ClimberMotorStates.OFF;

                CLIMBER_PARAMS.READY_POSITION = .7;
                CLIMBER_PARAMS.STOWED_STEP1_VALUE = .6;
                CLIMBER_PARAMS.STOWED_STEP2_VALUE = .53;
                CLIMBER_PARAMS.STOWED_STEP3_VALUE = .5;
                CLIMBER_PARAMS.STOWED_POSITION = .48;

                CLIMBER_PARAMS.ROBOT_DOWN_POWER = -0.8;
                CLIMBER_PARAMS.ROBOT_UP_POWER = 0.8;
                break;

            default:
                throw new IllegalArgumentException("Unknown robot type: " + robotType);
        }
    }

    public static class ClimberParameters {

        public ClimberArmStates CLIMBER_ARM_STARTING_STATE;
        public ClimberMotorStates CLIMBER_MOTOR_STARTING_STATE;

        public double READY_POSITION = 0;
        public double STOWED_STEP1_VALUE = .6;
        public double STOWED_STEP2_VALUE = .53;
        public double STOWED_STEP3_VALUE = .5;
        public double STOWED_POSITION = 1;

        public double ROBOT_DOWN_POWER = -0.8;
        public double ROBOT_UP_POWER = 0.8;
    }

    public static ClimberParameters CLIMBER_PARAMS = new ClimberParameters();

    public enum ClimberArmStates {
        STOWED,
        STOWED_STEP1,
        STOWED_STEP2,
        STOWED_STEP3,
        READY;
        public double getPosition() {
            switch (this) {
                case STOWED_STEP1:
                    return CLIMBER_PARAMS.STOWED_STEP1_VALUE;
                case STOWED_STEP2:
                    return CLIMBER_PARAMS.STOWED_STEP2_VALUE;
                case STOWED_STEP3:
                    return CLIMBER_PARAMS.STOWED_STEP3_VALUE;
                case STOWED:
                    return CLIMBER_PARAMS.STOWED_POSITION;
                case READY:
                    return CLIMBER_PARAMS.READY_POSITION;
                default:
                    throw new IllegalStateException("Unknown climber arm state: " + this);
            }
        }
    }

    public enum ClimberMotorStates {
        ROBOT_DOWN,
        ROBOT_UP,
        OFF;

        public double getPower() {
            switch (this) {
                case ROBOT_DOWN:
                    return CLIMBER_PARAMS.ROBOT_DOWN_POWER;
                case ROBOT_UP:
                    return CLIMBER_PARAMS.ROBOT_UP_POWER;
                case OFF:
                    return 0.0;
                default:
                    throw new IllegalStateException("Unknown climber motor state: " + this);
            }
        }
    }

    public Servo climberArm;
    public DcMotorEx climberMotor;
    public ClimberArmStates currentClimberArmState;
    public ClimberMotorStates currentClimberMotorState;

    public void setCurrentClimberMotorState(ClimberMotorStates state) {
        currentClimberMotorState = state;}
    public ClimberMotorStates getCurrentClimberMotorState() {return currentClimberMotorState;}

    public void setCurrentClimberArmState(ClimberArmStates state) {currentClimberArmState = state;}
    public ClimberArmStates getCurrentClimberArmState() {return currentClimberArmState;}

    public ClimberSubsystem(final HardwareMap hMap, Robot.RobotType robotType, final String climberArmName, final String climberMotorName) {
        configureParamsForRobotType(robotType);
        climberArm = hMap.servo.get(climberArmName);
        climberMotor = hMap.get(DcMotorEx.class, climberMotorName);
    }

    public void init() {
//        climberMotorInit();
//        climberArmInit();
    }

    private void climberArmInit() {
        currentClimberArmState = CLIMBER_PARAMS.CLIMBER_ARM_STARTING_STATE;
    }

    private void climberMotorInit() {
        currentClimberMotorState = CLIMBER_PARAMS.CLIMBER_MOTOR_STARTING_STATE;
        climberMotor.setDirection(DcMotor.Direction.REVERSE);
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentClimberMotorState = ClimberMotorStates.OFF;
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberMotor.setPower(0);
        climberMotor.setVelocity(0);
    }

    public void periodic(){
        MatchConfig.telemetryPacket.put("Climber Motor State", currentClimberMotorState);

        //Add the Climber Arm State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Climber Servo State", currentClimberArmState);
    }
}

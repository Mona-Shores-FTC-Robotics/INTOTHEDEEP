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
                CLIMBER_PARAMS.WINCH_MOTOR_STARTING_STATE = WinchMotorStates.OFF;

                CLIMBER_PARAMS.STOWED_POSITION = 0.5;
                CLIMBER_PARAMS.READY_POSITION = .7;

                CLIMBER_PARAMS.ROBOT_DOWN_POWER = -0.8;
                CLIMBER_PARAMS.ROBOT_UP_POWER = 0.8;
                break;

            case INTO_THE_DEEP_20245:
                CLIMBER_PARAMS.CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED;
                CLIMBER_PARAMS.WINCH_MOTOR_STARTING_STATE = WinchMotorStates.OFF;

                CLIMBER_PARAMS.STOWED_POSITION = 0.5;
                CLIMBER_PARAMS.READY_POSITION = .7;

                CLIMBER_PARAMS.ROBOT_DOWN_POWER = -0.8;
                CLIMBER_PARAMS.ROBOT_UP_POWER = 0.8;
                break;

            default:
                throw new IllegalArgumentException("Unknown robot type: " + robotType);
        }
    }

    public static class ClimberParameters {
        public ClimberArmStates CLIMBER_ARM_STARTING_STATE;
        public WinchMotorStates WINCH_MOTOR_STARTING_STATE;

        public double STOWED_POSITION;
        public double READY_POSITION;

        public double ROBOT_DOWN_POWER;
        public double ROBOT_UP_POWER;
    }

    public static ClimberParameters CLIMBER_PARAMS = new ClimberParameters();

    public enum ClimberArmStates {
        STOWED,
        READY;
        public double getPosition() {
            switch (this) {
                case STOWED:
                    return CLIMBER_PARAMS.STOWED_POSITION;
                case READY:
                    return CLIMBER_PARAMS.READY_POSITION;
                default:
                    throw new IllegalStateException("Unknown climber arm state: " + this);
            }
        }
    }

    public enum WinchMotorStates {
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
                    throw new IllegalStateException("Unknown winch motor state: " + this);
            }
        }
    }

    public Servo climberArm;
    public DcMotorEx winchMotor;
    public ClimberArmStates currentClimberArmState;
    public WinchMotorStates currentWinchMotorState;

    public void setCurrentWinchMotorState(WinchMotorStates state) {currentWinchMotorState = state;}
    public WinchMotorStates getCurrentWinchMotorState() {return currentWinchMotorState;}

    public void setCurrentClimberArmState(ClimberArmStates state) {currentClimberArmState = state;}
    public ClimberArmStates getCurrentClimberArmState() {return currentClimberArmState;}

    public ClimberSubsystem(final HardwareMap hMap, Robot.RobotType robotType, final String climberArmName, final String winchMotorName) {
        configureParamsForRobotType(robotType);
        climberArm = hMap.servo.get(climberArmName);
        winchMotor = hMap.get(DcMotorEx.class, winchMotorName);
    }

    public void init() {
        winchMotorInit();
        climberArmInit();
    }

    private void climberArmInit() {
        currentClimberArmState = CLIMBER_PARAMS.CLIMBER_ARM_STARTING_STATE;
    }

    private void winchMotorInit() {
        currentWinchMotorState = CLIMBER_PARAMS.WINCH_MOTOR_STARTING_STATE;
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentWinchMotorState = WinchMotorStates.OFF;
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setPower(0);
        winchMotor.setVelocity(0);
    }

    public void periodic(){
        //Add the Winch Motor State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Winch State", currentWinchMotorState);

        //Add the Climber Arm State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Climber Arm State", currentClimberArmState);
    }
}

package org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.End_Game;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class ClimberSubsystem extends SubsystemBase {

    public static class ClimberParameters  {

        public ClimberArmStates CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED;
        public WinchMotorStates WINCH_MOTOR_STARTING_STATE = WinchMotorStates.ROBOT_DOWN;

        public double STOWED_VALUE = .5;
        public double READY_VALUE = .7;

        public double ROBOT_DOWN_POWER = -.8;
        public double ROBOT_UP_POWER = .8;
    }

    public static ClimberParameters climberParameters = new ClimberParameters();

    public enum ClimberArmStates {
        STOWED (.5),
        READY (.8);
        public double position;
        ClimberArmStates(double p) {
            this.position = p;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public enum WinchMotorStates {
        ROBOT_DOWN(-.3),
        ROBOT_UP(1),
        OFF(0);

        public double power;
        WinchMotorStates(double p) {
            this.power = p;
        }
        void SetState(double p){
            this.power = p;
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

    public ClimberSubsystem(final HardwareMap hMap, final String climberArmName, final String winchMotorName) {
        climberArm = hMap.servo.get(climberArmName);
        winchMotor = hMap.get(DcMotorEx.class, winchMotorName);
    }

    public void init() {
        winchMotorInit();
        climberArmInit();
    }

    private void climberArmInit() {
        currentClimberArmState = climberParameters.CLIMBER_ARM_STARTING_STATE;
    }

    private void winchMotorInit() {
        currentWinchMotorState = climberParameters.WINCH_MOTOR_STARTING_STATE;
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentWinchMotorState = WinchMotorStates.OFF;
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setPower(0);
        winchMotor.setVelocity(0);
    }

    public void periodic(){
        //Save the values to the enum every loop so we can adjust in the dashboard
        ClimberArmStates.READY.SetState(climberParameters.READY_VALUE);
        ClimberArmStates.STOWED.SetState(climberParameters.STOWED_VALUE);
        WinchMotorStates.ROBOT_DOWN.SetState(climberParameters.ROBOT_DOWN_POWER);
        WinchMotorStates.ROBOT_UP.SetState(climberParameters.ROBOT_UP_POWER);

        //Add the Winch Motor State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("End Game Winch State", currentWinchMotorState);

        //Add the Climber Arm State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Climber Arm State", currentClimberArmState);
    }
}

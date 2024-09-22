package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class LiftSubsystem extends SubsystemBase {

    // Static instance of LiftParams
    public static LiftParams LIFT_PARAMS = new LiftParams();

    public static class LiftParams {
        public final int MAX_TARGET_TICKS = 2800;
        public final int MIN_TARGET_TICKS = 0;
        public int LIFT_HEIGHT_TICK_THRESHOLD = 45;
        public double TIMEOUT_TIME_SECONDS = 2;
        public double EXTENSION_LIFT_POWER = .6;
        public double RETRACTION_LIFT_POWER = .3;
        public double VEL_P=5, VEL_I=0, VEL_D=0, VEL_F=38;
        public double POS_P=5;
        public double SCALE_FACTOR_FOR_MANUAL_LIFT=150;
        public double LIFT_DEAD_ZONE_FOR_MANUAL_LIFT;

        public int MAX_HEIGHT_TICKS=2600;

        public int HOME_HEIGHT_TICKS=25;
        public int SAFE_HEIGHT_TICKS=125;

        public int HIGH_BASKET_TICKS=2500;
        public int LOW_BASKET_TICKS=1500;
        public int HIGH_CHAMBER_TICKS=2000;
        public int LOW_CHAMBER_TICKS=1400;

        public int HANG_TICK_DIFFERENCE = 200;
    }

    // Motor and state variables
    public DcMotorEx lift;
    private LiftStates currentState;
    private LiftStates targetState;
    private int currentTicks;
    private int targetTicks;
    private double power;


    // Constructor to initialize the lift motor
    public LiftSubsystem(final HardwareMap hMap, final String name) {
        lift = hMap.get(DcMotorEx.class, name);
    }

    public void init (){
        //This Direction works better because the string doesn't coil up in the other direction on the spool
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Apply PID coefficients from LIFT_PARAMS
        lift.setVelocityPIDFCoefficients(LIFT_PARAMS.VEL_P, LIFT_PARAMS.VEL_I, LIFT_PARAMS.VEL_D, LIFT_PARAMS.VEL_F);
        lift.setPositionPIDFCoefficients(LIFT_PARAMS.POS_P);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = LIFT_PARAMS.EXTENSION_LIFT_POWER;  // Set default power from params
        lift.setPower(power);

        currentState = LiftStates.ZERO;
        lift.setTargetPosition(currentState.ticks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void periodic(){
        // Update PID coefficients during runtime
        lift.setVelocityPIDFCoefficients(LIFT_PARAMS.VEL_P, LIFT_PARAMS.VEL_I, LIFT_PARAMS.VEL_D, LIFT_PARAMS.VEL_F);
        lift.setPositionPIDFCoefficients(LIFT_PARAMS.POS_P);

        // Update the current position of the lift
        currentTicks = lift.getCurrentPosition();

        // Telemetry for debugging
        MatchConfig.telemetryPacket.put("LiftSlide State", currentState);
        MatchConfig.telemetryPacket.put("LiftSlide Ticks", currentTicks);
        //        MatchConfig.telemetryPacket.put("LiftSlide Deliver Height", Robot.getInstance().getVisionSubsystem().getDeliverHeight());


        if (targetState != currentState) {
            MatchConfig.telemetryPacket.put("LiftSlide Target State", targetState);
            MatchConfig.telemetryPacket.put("LiftSlide Target Ticks", targetTicks);
        }

        // Dynamically update the lift height ticks based on liftHeights
        LiftStates.HOME.setLiftHeightTicks(LIFT_PARAMS.HOME_HEIGHT_TICKS);
        LiftStates.HIGH_BASKET.setLiftHeightTicks(LIFT_PARAMS.HIGH_BASKET_TICKS);
        LiftStates.LOW_BASKET.setLiftHeightTicks(LIFT_PARAMS.LOW_BASKET_TICKS);
        LiftStates.HIGH_CHAMBER.setLiftHeightTicks(LIFT_PARAMS.HIGH_CHAMBER_TICKS);
        LiftStates.LOW_CHAMBER.setLiftHeightTicks(LIFT_PARAMS.LOW_CHAMBER_TICKS);
        LiftStates.HANG_HIGH_CHAMBER.setLiftHeightTicks(LIFT_PARAMS.HIGH_CHAMBER_TICKS- LIFT_PARAMS.HANG_TICK_DIFFERENCE);
        LiftStates.HANG_LOW_CHAMBER.setLiftHeightTicks(LIFT_PARAMS.LOW_CHAMBER_TICKS- LIFT_PARAMS.HANG_TICK_DIFFERENCE);
        LiftStates.MAX.setLiftHeightTicks(LIFT_PARAMS.MAX_HEIGHT_TICKS);

    }

    public enum LiftStates {
        ZERO, SAFE, MAX, HIGH_BASKET, LOW_BASKET, HIGH_CHAMBER, LOW_CHAMBER, HOME, HANG_HIGH_CHAMBER, HANG_LOW_CHAMBER, MANUAL;
        public int ticks;

        static {
            ZERO.ticks = 0;
            MAX.ticks = LIFT_PARAMS.MAX_HEIGHT_TICKS;
            SAFE.ticks = LIFT_PARAMS.SAFE_HEIGHT_TICKS;
            HIGH_BASKET.ticks = LIFT_PARAMS.HIGH_BASKET_TICKS;
            LOW_BASKET.ticks = LIFT_PARAMS.LOW_BASKET_TICKS;
            HIGH_CHAMBER.ticks = LIFT_PARAMS.HIGH_CHAMBER_TICKS;
            HANG_HIGH_CHAMBER.ticks = LIFT_PARAMS.HIGH_CHAMBER_TICKS-LIFT_PARAMS.HANG_TICK_DIFFERENCE;
            LOW_CHAMBER.ticks = LIFT_PARAMS.LOW_CHAMBER_TICKS;
            HANG_LOW_CHAMBER.ticks = LIFT_PARAMS.LOW_CHAMBER_TICKS-LIFT_PARAMS.HANG_TICK_DIFFERENCE;
            HOME.ticks = LIFT_PARAMS.HOME_HEIGHT_TICKS;
        }
        public void setLiftHeightTicks(int t){
            this.ticks = t;
        }
    }
    public void setCurrentState(LiftStates state) {currentState = state;}
    public LiftStates getCurrentState() {return currentState;}
    public void setTargetState(LiftStates state) {targetState = state;}
    public LiftStates getTargetState() {return targetState;}
    public int getTargetTicks() {return targetTicks;}
    public void setTargetTicks(int ticks) {targetTicks=ticks;}
    public int getCurrentTicks() {
        return currentTicks;
    }
}
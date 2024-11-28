package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ConfigurableParameters;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;

@Config
public class SampleLinearActuatorSubsystem extends SubsystemBase {

    public static class ActuatorParams extends ConfigurableParameters {
        public double FLIP_UP_INCREMENT_TIME =1.0 ;
        public double FLIP_UP_POSITION;
        public double FLIP_DOWN_POSITION;
        public double FLIP_UP_DELAY_TIME_MS;
        public double MANUAL_MOVEMENT_SCALAR = Double.NaN;
        public double NORMAL_POWER = Double.NaN;
        public double POWER_FOR_SLOW_DEPLOYMENT = Double.NaN;
        public double DEAD_ZONE_FOR_MANUAL_ACTUATION = Double.NaN;

        public double FULL_DEPLOYMENT_TIME_MS = Double.NaN;
        public double PARTIAL_DEPLOYMENT_TIME_MS = Double.NaN;
        public double FULL_RETRACTION_TIME_MS = Double.NaN;
        public double PARTIAL_RETRACTION_TIME_MS = Double.NaN;

        @Override
        public void loadDefaultsForRobotType(Robot.RobotType robotType) {
            if (haveRobotSpecificParametersBeenLoaded()) return;

            switch (robotType) {
                case INTO_THE_DEEP_19429:
                    ACTUATOR_PARAMS.MANUAL_MOVEMENT_SCALAR = 0.8;
                    ACTUATOR_PARAMS.NORMAL_POWER = 1.0;
                    ACTUATOR_PARAMS.POWER_FOR_SLOW_DEPLOYMENT = 0.4;
                    ACTUATOR_PARAMS.DEAD_ZONE_FOR_MANUAL_ACTUATION = 0.1;

                    ACTUATOR_PARAMS.FULL_DEPLOYMENT_TIME_MS = 600;
                    ACTUATOR_PARAMS.PARTIAL_DEPLOYMENT_TIME_MS = 140;
                    ACTUATOR_PARAMS.FULL_RETRACTION_TIME_MS = 700;
                    ACTUATOR_PARAMS.PARTIAL_RETRACTION_TIME_MS = 100;
                    FLIP_UP_DELAY_TIME_MS = 250;
                    FLIP_UP_POSITION= .4;
                    FLIP_DOWN_POSITION =0.7;
                    break;

                case INTO_THE_DEEP_20245:
                    ACTUATOR_PARAMS.MANUAL_MOVEMENT_SCALAR = 0.8;
                    ACTUATOR_PARAMS.NORMAL_POWER = 0.7;
                    ACTUATOR_PARAMS.POWER_FOR_SLOW_DEPLOYMENT = 0.4;
                    ACTUATOR_PARAMS.DEAD_ZONE_FOR_MANUAL_ACTUATION = 0.1;

                    ACTUATOR_PARAMS.FULL_DEPLOYMENT_TIME_MS = 600;
                    ACTUATOR_PARAMS.PARTIAL_DEPLOYMENT_TIME_MS = 175;
                    ACTUATOR_PARAMS.FULL_RETRACTION_TIME_MS = 700;
                    ACTUATOR_PARAMS.PARTIAL_RETRACTION_TIME_MS = 100;

                    FLIP_UP_DELAY_TIME_MS = 250;
                    FLIP_UP_POSITION= .4;
                    FLIP_DOWN_POSITION =0.7;
                    break;

                default:
                    throw new IllegalArgumentException("Unknown robot type: " + robotType);
            }
            markRobotSpecificParametersLoaded();
        }
    }

    public static ActuatorParams ACTUATOR_PARAMS = new ActuatorParams();

    public enum SampleActuatorStates {
        PARTIALLY_RETRACTED_AFTER_EJECTION,
        FULLY_RETRACTED,
        PARTIALLY_RETRACTING_AFTER_EJECTING,
        FULLY_RETRACTING,

        PARTIALLY_DEPLOYING,
        PARTIALLY_DEPLOYED,

        WAITING_FOR_FLIP_UP,

        MANUAL,
        UNKNOWN
    }

    private final DcMotorEx sampleActuator;
    private SampleActuatorStates currentState;
    private double currentPower;
    private DigitalChannel retractedLimitSwitch;
    int currentTicks;



    private final Servo sampleIntakeFlipperServo;

    ElapsedTime actuatorTimer = new ElapsedTime();
    ElapsedTime flipUpTimer = new ElapsedTime();

    private double targetFlipperPosition;
    private double currentFlipperPosition;
    private double flipperStepIncrement;
    private boolean movingToTarget = false;
    ElapsedTime flipUpIncrementTimer = new ElapsedTime();

    // Constructor with limit switch
    public SampleLinearActuatorSubsystem(HardwareMap hardwareMap, Robot.RobotType robotType, String actuatorMotorName, String sampleIntakeFlipperServoName) {
        ACTUATOR_PARAMS.loadDefaultsForRobotType(robotType);
        sampleActuator = hardwareMap.get(DcMotorEx.class, actuatorMotorName);
        sampleActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sampleActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sampleActuator.setDirection(DcMotorEx.Direction.FORWARD);
        sampleActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentState = SampleActuatorStates.FULLY_RETRACTED;
        sampleIntakeFlipperServo=hardwareMap.get(Servo.class,sampleIntakeFlipperServoName);

        // Initialize the limit switch if the name is provided
//        if (limitSwitchName != null && !limitSwitchName.isEmpty()) {
//            retractedLimitSwitch = hardwareMap.get(DigitalChannel.class, limitSwitchName);
//            retractedLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
//        }
    }

    // Initialize actuator motor with encoders and PID configuration
    public void init() {
        // Initialize the current and target states to retracted
        currentState = SampleActuatorStates.FULLY_RETRACTED;
        sampleActuator.setPower(0);

    }

    @Override
    public void periodic() {
        // Cache the current actuator position and target ticks
        currentTicks = sampleActuator.getCurrentPosition();
        currentPower = sampleActuator.getPower();

        // Handle bucket movement
        if (movingToTarget && flipUpIncrementTimer.milliseconds() > ACTUATOR_PARAMS.FLIP_UP_INCREMENT_TIME) { // Adjust delay as needed
            currentFlipperPosition += flipperStepIncrement;
            sampleIntakeFlipperServo.setPosition(currentFlipperPosition);
            flipUpIncrementTimer.reset();

            if (currentFlipperPosition >= targetFlipperPosition )
            {
                sampleIntakeFlipperServo.setPosition(ACTUATOR_PARAMS.FLIP_DOWN_POSITION);
                movingToTarget = false;
            }
        }

        switch (currentState)
        {
            case WAITING_FOR_FLIP_UP:
                if (flipUpTimer.milliseconds()>= ACTUATOR_PARAMS.FLIP_UP_DELAY_TIME_MS)
                {
                    fullyRetract();
                }
                break;
            case PARTIALLY_DEPLOYING:
                if (actuatorTimer.milliseconds() >= ACTUATOR_PARAMS.PARTIAL_DEPLOYMENT_TIME_MS) {
                    stopActuator();
                    setCurrentState(SampleActuatorStates.PARTIALLY_DEPLOYED);
                }
                break;

            case PARTIALLY_RETRACTING_AFTER_EJECTING:
                if (actuatorTimer.milliseconds() >= ACTUATOR_PARAMS.PARTIAL_RETRACTION_TIME_MS) {
                    stopActuator();
                    if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE))
                    {
                        Robot.getInstance().getSampleIntakeSubsystem().setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    }
                    setCurrentState(SampleActuatorStates.PARTIALLY_RETRACTED_AFTER_EJECTION);
                }
            case FULLY_RETRACTING:
                if (actuatorTimer.milliseconds() >= ACTUATOR_PARAMS.FULL_RETRACTION_TIME_MS) {
                    stopActuator();
                    setCurrentState(SampleActuatorStates.FULLY_RETRACTED);
                }
                break;
            case PARTIALLY_DEPLOYED:
            case FULLY_RETRACTED:
            case MANUAL:
            case UNKNOWN:
            case PARTIALLY_RETRACTED_AFTER_EJECTION:
                //do nothing
                break;
        }
        updateDashboardTelemetry();  // Update telemetry each loop
    }


    public void partiallyRetractAndIntakeOn() {
        currentState=SampleActuatorStates.PARTIALLY_RETRACTING_AFTER_EJECTING;
        runWithoutEncodersReverse();
        actuatorTimer.reset();
    }

    public void fullyRetract() {
        currentState=SampleActuatorStates.FULLY_RETRACTING;
        runWithoutEncodersReverse();
        actuatorTimer.reset();
    }

    public void partiallyDeploy() {
        currentState=SampleActuatorStates.PARTIALLY_DEPLOYING;
        runWithoutEncodersForward();
        actuatorTimer.reset();
    }



    // Method to power the motor on in one direction without encoders
    public void runWithoutEncodersForward() {
        moveActuator(ACTUATOR_PARAMS.NORMAL_POWER);
    }

    // Method to power the motor on in one direction without encoders
    public void runWithoutEncodersForwardSlowly() {
        moveActuator(ACTUATOR_PARAMS.POWER_FOR_SLOW_DEPLOYMENT);
    }

    // Method to power the motor on in reverse without encoders
    public void runWithoutEncodersReverse() {
        moveActuator(-ACTUATOR_PARAMS.NORMAL_POWER);
    }

    // Add a method to handle manual input for the lift
    public void manualMove(double actuatorInput) {
        // Set the actuator state to MANUAL
        currentState=SampleActuatorStates.MANUAL;
        actuatorInput *= ACTUATOR_PARAMS.MANUAL_MOVEMENT_SCALAR;
        double actuatorManualPower = Range.clip(actuatorInput, -.5, .5);
        sampleActuator.setPower(actuatorManualPower);
    }

    public void setCurrentState(SampleActuatorStates state) {
        currentState = state;
    }

    // Method to check if the actuator is fully retracted
    public boolean isFullyRetracted() {
        if (retractedLimitSwitch == null) {
            System.out.println("Limit switch not configured; assuming fully retracted.");
            return true;
        }   return !retractedLimitSwitch.getState();  // Assuming switch triggers when low
    }

    public SampleActuatorStates getCurrentState() {
        return currentState;
    }

    // Method to stop the motor
    public void stopActuator() {
        currentPower = 0;
        sampleActuator.setPower(0);
    }

    // Apply power to move the actuator in (positive power) or out (negative power)
    private void moveActuator(double power) {
        currentPower = Range.clip(power, -1.0, 1.0);  // Ensure power is within valid range
        sampleActuator.setPower(currentPower);
    }

    // Update dashboard telemetry with actuator state
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Sample Actuator/Current State", currentState.toString());
        MatchConfig.telemetryPacket.put("Sample Actuator/Current Position Ticks", currentTicks);
    }

    // Compact telemetry display for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("%s | Actuator Position: %d", currentState != null ? currentState : "MANUAL_ACTUATOR", currentTicks);
        telemetry.addLine(telemetryData);
    }

    // Verbose telemetry display
    public void displayVerboseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Sample Actuator Current State", currentState);
        telemetry.addData("Sample Actuator Current Position Ticks", currentTicks);
        telemetry.addData("Sample Actuator Motor Power", currentPower);
    }

    // Verbose telemetry display
    public void flipSampleIntakeUpAndRetract() {
        currentState=SampleActuatorStates.WAITING_FOR_FLIP_UP;
        flipUpTimer.reset(); // once this timer hits a timer then do the retract in the periodic
        sampleIntakeFlipperServo.setPosition(ACTUATOR_PARAMS.FLIP_UP_POSITION);
    }
    public void flipSampleIntakeDown() {
        sampleIntakeFlipperServo.setPosition(ACTUATOR_PARAMS.FLIP_DOWN_POSITION);
    }

    public void flipSampleIntakeUp() {
        sampleIntakeFlipperServo.setPosition(ACTUATOR_PARAMS.FLIP_UP_POSITION);
    }

    public void setFlipperTargetPositionWithSteps(double targetPosition, int numSteps) {
        targetFlipperPosition = targetPosition;
        currentFlipperPosition = sampleIntakeFlipperServo.getPosition();  // Starting position
        flipperStepIncrement = (targetFlipperPosition - currentFlipperPosition) / numSteps;
        movingToTarget = true;
        flipUpIncrementTimer.reset();  // Start timing
    }

}

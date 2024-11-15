package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem.SpecimenArmStates.CCW_ARM_HOME;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem.SpecimenArmStates.CW_ARM_HOME;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class SpecimenArmSubsystem extends SubsystemBase {


    public static class SpecimenArmParams {

        //Flip parameters
        public double CCW_FLIP_TIME_MS = 250;
        public double CONSTANT_POWER_FOR_CCW_FLIP = 1.0;
        public double CW_FLIP_TIME_MS = 400;
        public double CONSTANT_POWER_FOR_CW_FLIP = -0.9;
        public double ZERO_POWER_SETTLE_TIME_MS = 225;

        //GamePad parameters
        public double STICK_SCALE_FACTOR = 5;
        public double DEAD_ZONE = 0.05;

        //PID parameters
        public double P = 0.016, I = .01, D = 0; // PID coefficients
        public double ANGLE_TOLERANCE_THRESHOLD_DEGREES = .5;

        //Arm Feedforward parameters
        public double kS = 0, kCos = 0.13, kV = 0, kA = 0; // Feedforward coefficients

        //Maximum Power for clipping motor output
        public double MAX_POWER = 0.7;

        //Preset Angles
        public double CCW_HOME = 245.69;
        public double SPECIMEN_PICKUP_ANGLE = 211.0;
        //210 is bad
        //211 worked
        //215 most we can do
        public double CW_HOME = 38.79;

        // Motion Profile Parameters
        public double TIMEOUT_TIME_SECONDS = 5;

        public double ENCODER_OFFSET=123.28;
    }
    public enum SpecimenArmStates {
        CCW_ARM_HOME,
        CW_ARM_HOME,
        SPECIMEN_PICKUP,
        FLIPPING_TO_CCW,
        FLIPPING_TO_CW,
        ZERO_POWER_AT_CCW_ARM_HOME,
        ZERO_POWER_AT_CW_ARM_HOME;
        private double angle;

        static {
            CCW_ARM_HOME.angle = SPECIMEN_ARM_PARAMS.CCW_HOME;
            SPECIMEN_PICKUP.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE;
            CW_ARM_HOME.angle = SPECIMEN_ARM_PARAMS.CW_HOME;
        }

        public void setArmAngle(double t) {
            this.angle = t;
        }

        public double getArmAngle() {
            return this.angle;
        }
    }

    public static SpecimenArmParams SPECIMEN_ARM_PARAMS = new SpecimenArmParams();
    public DcMotorEx arm;

    // Arm Encoder Variables
    private double currentVelocity;
    private double currentTicks;

    //State Variables
    private SpecimenArmStates currentState;

    //Angle Variables
    private double currentAngleDegrees;
    private double targetAngleDegrees;



    //Feedforward
    private ArmFeedforward armFeedforward;
    private double feedforwardPower;

    // Detect our battery voltage to scale our feedforward parameters
    private static final double NOMINAL_VOLTAGE = 12.0;
    VoltageSensor voltageSensor;


    // PID Controller
    public PIDController pidController;
    private double pidPower;
    private double totalPower;

    private static final double DEGREES_PER_US = 360.0 / 1024.0; // Encoder calibration factor

    //Store the clipped power as a variable for telemetry
    private double clippedPower;


    // Timer to track elapsed time for motion profile
    private final ElapsedTime flipArmTimer;
    private final ElapsedTime zeroPowerTimer;


    private double prev_kS = SPECIMEN_ARM_PARAMS.kS;
    private double prev_kCos = SPECIMEN_ARM_PARAMS.kCos;
    private double prev_kV = SPECIMEN_ARM_PARAMS.kV;
    private double prev_kA = SPECIMEN_ARM_PARAMS.kA;

    // OctoQuad encoder instance for the arm
    private final OctoQuad octoquad;
    private final int armEncoderChannel = 4; // Specify the OctoQuad channel for the arm encoder

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        currentState = CCW_ARM_HOME;
        flipArmTimer = new ElapsedTime();
        zeroPowerTimer = new ElapsedTime();

        // Initialize the OctoQuad encoder for arm positioning
        octoquad = hMap.get(OctoQuad.class, "octoquad");
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_PULSE_WIDTH);
        octoquad.setSingleChannelPulseWidthParams(armEncoderChannel, new OctoQuad.ChannelPulseWidthParams(1, 1024));
    }

    public void init() {
        voltageSensor = Robot.getInstance().getActiveOpMode().hardwareMap.voltageSensor.iterator().next();
        double batteryVoltage = voltageSensor.getVoltage();
        double voltageScale = NOMINAL_VOLTAGE / batteryVoltage;

        armFeedforward = new ArmFeedforward(
                SPECIMEN_ARM_PARAMS.kS * voltageScale,
                SPECIMEN_ARM_PARAMS.kCos,
                SPECIMEN_ARM_PARAMS.kV * voltageScale,
                SPECIMEN_ARM_PARAMS.kA * voltageScale
        );

        pidController = new PIDController(SPECIMEN_ARM_PARAMS.P, SPECIMEN_ARM_PARAMS.I, SPECIMEN_ARM_PARAMS.D);
        pidController.setTolerance(SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES);
        pidController.reset();
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        currentState = CCW_ARM_HOME;
        currentAngleDegrees= SPECIMEN_ARM_PARAMS.CCW_HOME;
        targetAngleDegrees = SPECIMEN_ARM_PARAMS.CCW_HOME;
        pidController.setSetPoint(targetAngleDegrees);
    }

    @Override
    public void periodic() {
        // Retrieve current position from the encoder
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();
        octoquad.readAllEncoderData(encoderDataBlock);
        currentTicks = encoderDataBlock.positions[armEncoderChannel]; // Position in microseconds (uS)
        currentVelocity = encoderDataBlock.velocities[armEncoderChannel]; // Position in microseconds (uS)
        currentAngleDegrees = calculateCurrentArmAngleInDegrees();

        switch (currentState) {
            case FLIPPING_TO_CW:
                if (flipArmTimer.milliseconds() > SPECIMEN_ARM_PARAMS.CW_FLIP_TIME_MS) {
                    arm.setPower(0);
                    setCurrentState(SpecimenArmStates.ZERO_POWER_AT_CW_ARM_HOME);
                    zeroPowerTimer.reset();
                }
                break;
            case FLIPPING_TO_CCW:
                if (flipArmTimer.milliseconds() > SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS) {
                    arm.setPower(0);
                    setCurrentState(SpecimenArmStates.ZERO_POWER_AT_CCW_ARM_HOME);
                    zeroPowerTimer.reset();
                }
                break;
            case ZERO_POWER_AT_CCW_ARM_HOME:
                if (zeroPowerTimer.milliseconds() > SPECIMEN_ARM_PARAMS.ZERO_POWER_SETTLE_TIME_MS) {
                  setCurrentState(CCW_ARM_HOME);
                }
                break;
            case ZERO_POWER_AT_CW_ARM_HOME:
                if (zeroPowerTimer.milliseconds() > SPECIMEN_ARM_PARAMS.ZERO_POWER_SETTLE_TIME_MS) {
                    setCurrentState(CW_ARM_HOME);
                }
                break;

            case CCW_ARM_HOME:
            case CW_ARM_HOME:
            case SPECIMEN_PICKUP:
            default:
//                 Check if movement is needed based on the target angle
                if (Math.abs(targetAngleDegrees - currentAngleDegrees) > SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES) {
                    moveToTargetAngle();  // Move towards the target if outside tolerance
                } else {
                    maintainPosition();  // Hold the position if within tolerance
                }
                break;
        }
        updateParameters();
        updateDashboardTelemetry();
    }

    public void flipCCWFast() {
        flipArmTimer.reset();
        currentState=SpecimenArmStates.FLIPPING_TO_CCW;
        targetAngleDegrees=CCW_ARM_HOME.angle;
        pidController.setSetPoint(SpecimenArmStates.CCW_ARM_HOME.angle);
        arm.setPower(SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CCW_FLIP);
    }

    public void flipCWFast() {
        flipArmTimer.reset();
        currentState=SpecimenArmStates.FLIPPING_TO_CW;
        targetAngleDegrees= CW_ARM_HOME.angle;
        pidController.setSetPoint(CW_ARM_HOME.angle);
        arm.setPower(SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CW_FLIP);
    }

    public void fallToCCW() {
        arm.setPower(0);
        setCurrentState(SpecimenArmStates.ZERO_POWER_AT_CCW_ARM_HOME);
        zeroPowerTimer.reset();
    }

    public void gotoPickupAngle() {

        setCurrentState(SpecimenArmStates.SPECIMEN_PICKUP);
    }

    public void setManualTargetAngle(double armInput) {
            // Calculate the change in angle based on input and scale factor
            double deltaAngle = armInput * SPECIMEN_ARM_PARAMS.STICK_SCALE_FACTOR;

            // Calculate the new target angle based on the target angle
            targetAngleDegrees += deltaAngle;

            // Clip target angle to within allowed range
            targetAngleDegrees = Range.clip(targetAngleDegrees, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);

            // Update the PID controller's setPoint with the new target angle
            pidController.setSetPoint(targetAngleDegrees);
    }

    public void setTargetAngle(SpecimenArmStates state) {
        currentState=state;
        targetAngleDegrees = state.getArmAngle();
        pidController.setSetPoint(targetAngleDegrees);
    }

    private void moveToTargetAngle() {
        // Calculate PID output
        pidPower = pidController.calculate(currentAngleDegrees);

        // Calculate feedforward output
        feedforwardPower = armFeedforward.calculate(
                Math.toRadians(currentAngleDegrees),
                0,
                0
        );

        // Combine PID and feedforward control efforts
        totalPower = pidPower + feedforwardPower;

        // Clip the total power to allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Apply motor power
        arm.setPower(clippedPower);
    }

    private void maintainPosition() {
        // PID control for position
        pidPower = pidController.calculate (currentAngleDegrees); // Pass current position, not error
        feedforwardPower = armFeedforward.calculate (
                Math.toRadians (currentAngleDegrees),
               0,
                0
        );
        totalPower =  feedforwardPower+pidPower;
        clippedPower = Range.clip (totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);
        arm.setPower (clippedPower);
    }

    public SpecimenArmStates getCurrentState() {
        return currentState;
    }
    public void setCurrentState(SpecimenArmStates state) {
        targetAngleDegrees = state.getArmAngle();
        pidController.setSetPoint(state.getArmAngle());
        currentState = state;
    }

private double calculateCurrentArmAngleInDegrees() {
    // Convert ticks to degrees, apply offset, and reverse direction if needed
    double rawDegrees = currentTicks * DEGREES_PER_US;
    double adjustedDegrees = rawDegrees - SPECIMEN_ARM_PARAMS.ENCODER_OFFSET;

    // Normalize to 0-360 range
    adjustedDegrees = (adjustedDegrees % 360 + 360) % 360;

    return adjustedDegrees;
}


    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }
    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }
    public void updateParameters() {
        updateArmAngles(CCW_ARM_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);
        updateArmAngles(CW_ARM_HOME, SPECIMEN_ARM_PARAMS.CW_HOME);
        updateArmAngles(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE);
        updatePIDCoefficients();

        // Re-instantiate armFeedforward with updated parameters from dashboard

        if (SPECIMEN_ARM_PARAMS.kS != prev_kS ||
                SPECIMEN_ARM_PARAMS.kCos != prev_kCos ||
                SPECIMEN_ARM_PARAMS.kV != prev_kV ||
                SPECIMEN_ARM_PARAMS.kA != prev_kA) {

            double batteryVoltage = voltageSensor.getVoltage();
            double voltageScale = NOMINAL_VOLTAGE / batteryVoltage;

            // Update armFeedforward with new parameters
            armFeedforward = new ArmFeedforward(
                    SPECIMEN_ARM_PARAMS.kS * voltageScale,
                    SPECIMEN_ARM_PARAMS.kCos,
                    SPECIMEN_ARM_PARAMS.kV * voltageScale,
                    SPECIMEN_ARM_PARAMS.kA * voltageScale
            );

            // Update the previous values to the current ones
            prev_kS = SPECIMEN_ARM_PARAMS.kS;
            prev_kCos = SPECIMEN_ARM_PARAMS.kCos;
            prev_kV = SPECIMEN_ARM_PARAMS.kV;
            prev_kA = SPECIMEN_ARM_PARAMS.kA;
        }
    }
    private void updateArmAngles(SpecimenArmStates armState, double newArmAngle) {
        if (armState.getArmAngle() != newArmAngle) {
            armState.setArmAngle(newArmAngle);
        }
    }
    private void updatePIDCoefficients() {
        // Get the latest PID values from parameters
        double p = SPECIMEN_ARM_PARAMS.P;
        double i = SPECIMEN_ARM_PARAMS.I;
        double d = SPECIMEN_ARM_PARAMS.D;

        // Only update the PID controller if there's a change
        if (pidController.getP() != p || pidController.getI() != i || pidController.getD() != d) {
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
        }
    }

    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("%s | Angle: %.2f", currentState, currentAngleDegrees);
        telemetry.addLine(telemetryData);
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("specimenArm/Current State", currentState);
    }

    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        // Display state and target state information on one line
        String stateOverview = String.format("Specimen Arm State: %s", currentState);
        MatchConfig.telemetryPacket.addLine(stateOverview);

        // Add power overview on its own line
        String powerSummary = String.format("PID: %.2f | FF: %.2f | Clipped: %.2f", pidPower, feedforwardPower, clippedPower);
        MatchConfig.telemetryPacket.addLine(powerSummary);

        MatchConfig.telemetryPacket.put("SpecimenArm/angles/Current Arm Angle", String.format("%.2f", currentAngleDegrees));
        MatchConfig.telemetryPacket.put("SpecimenArm/angles/Target Arm Angle", String.format("%.2f", targetAngleDegrees));

        MatchConfig.telemetryPacket.put("SpecimenArm/Current Arm Velocity", String.format("%.2f", currentVelocity));
    }
}

package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem.SpecimenArmStates.CCW_ARM_HOME;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem.SpecimenArmStates.CW_ARM_HOME;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ConfigurableParameters;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SpecimenArmPowerMessage;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SpecimenArmStateMessage;

@Config
public class SpecimenArmSubsystem extends SubsystemBase {

    public static class SpecimenArmParams extends ConfigurableParameters {
        // Flip parameters
        public double CCW_FLIP_TIME_MS = Double.NaN;
        public double REVERSE_FLIP_TIME_MS = Double.NaN;
        public double CONSTANT_POWER_FOR_CCW_FLIP = Double.NaN;
        public double CW_FLIP_TIME_MS = Double.NaN;
        public double CONSTANT_POWER_FOR_CW_FLIP = Double.NaN;
        public double ZERO_POWER_SETTLE_TIME_MS = Double.NaN;

        // GamePad parameters
        public double STICK_SCALE_FACTOR = Double.NaN;
        public double DEAD_ZONE = Double.NaN;

        // PID parameters
        public double P = Double.NaN;
        public double I = Double.NaN;
        public double D = Double.NaN;
        public double ANGLE_TOLERANCE_THRESHOLD_DEGREES = Double.NaN;

        // Arm Feedforward parameters
        public double kS = Double.NaN;
        public double kCos = Double.NaN;
        public double kV = Double.NaN;
        public double kA = Double.NaN;

        // Maximum Power
        public double MAX_POWER = Double.NaN;

        // Preset Angles
        public double CCW_HOME = Double.NaN;
        public double CCW_FLIP_ARM_TARGET_ANGLE = Double.NaN;
        public double SPECIMEN_PICKUP_ANGLE = Double.NaN;
        private double DEFAULT_PICKUP_ANGLE;
        private double MAX_PICKUP_ANGLE_ADJUSTMENT;

        public double CW_HOME = Double.NaN;

        // Motion Profile Parameters
        public double RAMP_UP_TIME_MILLISECONDS =  Double.NaN;
        public double TIMEOUT_TIME_SECONDS = Double.NaN;
        public double ENCODER_OFFSET = Double.NaN;

        @Override
        public void loadDefaultsForRobotType(Robot.RobotType robotType) {
            if (haveRobotSpecificParametersBeenLoaded()) return;

            switch (robotType) {
                case INTO_THE_DEEP_19429:
                    // Flip parameters
                    SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS = 575;
                    SPECIMEN_ARM_PARAMS.REVERSE_FLIP_TIME_MS = SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS - 100;
                    SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CCW_FLIP = 1.0;
                    SPECIMEN_ARM_PARAMS.CW_FLIP_TIME_MS = 550;
                    SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CW_FLIP = - 1.0;
                    SPECIMEN_ARM_PARAMS.ZERO_POWER_SETTLE_TIME_MS = 550;

                    // GamePad parameters
                    SPECIMEN_ARM_PARAMS.STICK_SCALE_FACTOR = 5;
                    SPECIMEN_ARM_PARAMS.DEAD_ZONE = 0.05;

                    // PID parameters
                    SPECIMEN_ARM_PARAMS.P = 0.0044;
                    SPECIMEN_ARM_PARAMS.I = .025;
                    SPECIMEN_ARM_PARAMS.D = .0001;
                    SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES = 0.5;

                    // Arm Feedforward parameters
                    SPECIMEN_ARM_PARAMS.kS = 0;
                    SPECIMEN_ARM_PARAMS.kCos = 0.2;
                    SPECIMEN_ARM_PARAMS.kV = 0;
                    SPECIMEN_ARM_PARAMS.kA = 0;

                    // Maximum Power
                    SPECIMEN_ARM_PARAMS.MAX_POWER = 1.0;

                    // Preset Angles
                    SPECIMEN_ARM_PARAMS.CCW_HOME = 243.0;
                    SPECIMEN_ARM_PARAMS.CCW_FLIP_ARM_TARGET_ANGLE = 100;
                    SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE = 214;
                    SPECIMEN_ARM_PARAMS.CW_HOME = 38.79;

                    DEFAULT_PICKUP_ANGLE = SPECIMEN_PICKUP_ANGLE;
                    MAX_PICKUP_ANGLE_ADJUSTMENT=10;

                    // Motion Profile Parameters
                    SPECIMEN_ARM_PARAMS.RAMP_UP_TIME_MILLISECONDS = 250;
                    SPECIMEN_ARM_PARAMS.TIMEOUT_TIME_SECONDS = 1.5;

                    // Encoder Offset
                    SPECIMEN_ARM_PARAMS.ENCODER_OFFSET = 249.16;
                    break;

                case INTO_THE_DEEP_20245:
                    // Flip parameters
                    SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS = 575;
                    SPECIMEN_ARM_PARAMS.REVERSE_FLIP_TIME_MS = SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS - 100;
                    SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CCW_FLIP = 1.0;
                    SPECIMEN_ARM_PARAMS.CW_FLIP_TIME_MS = 550;
                    SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CW_FLIP = - 1.0;
                    SPECIMEN_ARM_PARAMS.ZERO_POWER_SETTLE_TIME_MS = 550;

                    // GamePad parameters
                    SPECIMEN_ARM_PARAMS.STICK_SCALE_FACTOR = 5;
                    SPECIMEN_ARM_PARAMS.DEAD_ZONE = 0.05;

                    // PID parameters
                    SPECIMEN_ARM_PARAMS.P = 0.0044;
                    SPECIMEN_ARM_PARAMS.I = 0.10;
                    SPECIMEN_ARM_PARAMS.D = 0.0001;
                    SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES = 0.5;

                    // Arm Feedforward parameters
                    SPECIMEN_ARM_PARAMS.kS = 0;
                    SPECIMEN_ARM_PARAMS.kCos = 0.2;
                    SPECIMEN_ARM_PARAMS.kV = 0;
                    SPECIMEN_ARM_PARAMS.kA = 0;

                    // Maximum Power
                    SPECIMEN_ARM_PARAMS.MAX_POWER = 1.0;

                    // Preset Angles
                    SPECIMEN_ARM_PARAMS.CCW_HOME = 243.0;
                    SPECIMEN_ARM_PARAMS.CCW_FLIP_ARM_TARGET_ANGLE = 100;
                    SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE = 222.8;
                    SPECIMEN_ARM_PARAMS.CW_HOME = 38.79;

                    // Motion Profile Parameters
                    SPECIMEN_ARM_PARAMS.RAMP_UP_TIME_MILLISECONDS = 250;
                    SPECIMEN_ARM_PARAMS.TIMEOUT_TIME_SECONDS = 1.5;

                    // Encoder Offset
                    SPECIMEN_ARM_PARAMS.ENCODER_OFFSET = 300;
                    break;

                default:
                    throw new IllegalArgumentException("Unknown robot type: " + robotType);
            }
            markRobotSpecificParametersLoaded();
        }
    }

    public enum SpecimenArmStates {
        CCW_ARM_HOME,
        CW_ARM_HOME,
        SPECIMEN_PICKUP,
        FLIPPING_TO_CCW,
        FLIPPING_TO_CW,
        ZERO_POWER_AT_CCW_ARM_HOME,
        ZERO_POWER_AT_CW_ARM_HOME,
        ROTATING_CCW_TO_TARGET_ANGLE; // New state for constant power rotation

        public double getArmAngle() {
            switch (this) {
                case CCW_ARM_HOME:
                case ZERO_POWER_AT_CCW_ARM_HOME:
                    return SPECIMEN_ARM_PARAMS.CCW_HOME;
                case CW_ARM_HOME:
                case ZERO_POWER_AT_CW_ARM_HOME:
                    return SPECIMEN_ARM_PARAMS.CW_HOME;
                case SPECIMEN_PICKUP:
                    return SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE;
                default:
                    throw new IllegalStateException("Angle not defined for state: " + this);
            }
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

    // Octoquad encoder instance for the arm
    private final OctoQuad octoquad;
    private final int armEncoderChannel = 4; // Specify the Octoquad channel for the arm encoder

    public SpecimenArmSubsystem(final HardwareMap hMap , Robot.RobotType robotType , final String name) {
        // Initialize parameters based on robot type
        SPECIMEN_ARM_PARAMS.loadDefaultsForRobotType(robotType); // Configure parameters for this robot type

        arm = hMap.get(DcMotorEx.class , name);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        currentState = CCW_ARM_HOME;
        flipArmTimer = new ElapsedTime();
        zeroPowerTimer = new ElapsedTime();

        // Initialize the Octoquad encoder for arm positioning
        octoquad = hMap.get(OctoQuad.class , "octoquad");
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_PULSE_WIDTH);
        octoquad.setSingleChannelPulseWidthParams(armEncoderChannel , new OctoQuad.ChannelPulseWidthParams(1 , 1024));
    }

    public void init() {
        voltageSensor = Robot.getInstance().getActiveOpMode().hardwareMap.voltageSensor.iterator().next();
        double batteryVoltage = voltageSensor.getVoltage();
        double voltageScale = NOMINAL_VOLTAGE / batteryVoltage;

        armFeedforward = new ArmFeedforward(
                SPECIMEN_ARM_PARAMS.kS * voltageScale ,
                SPECIMEN_ARM_PARAMS.kCos ,
                SPECIMEN_ARM_PARAMS.kV * voltageScale ,
                SPECIMEN_ARM_PARAMS.kA * voltageScale
        );

        pidController = new PIDController(SPECIMEN_ARM_PARAMS.P , SPECIMEN_ARM_PARAMS.I , SPECIMEN_ARM_PARAMS.D);
        pidController.setTolerance(SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES);
        pidController.reset();
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        currentState = CCW_ARM_HOME;
        currentAngleDegrees = SPECIMEN_ARM_PARAMS.CCW_HOME;
        targetAngleDegrees = SPECIMEN_ARM_PARAMS.CCW_HOME;
        pidController.setSetPoint(targetAngleDegrees);
    }

    @Override
    public void periodic() {
        // Retrieve current position from the encoder
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();
        octoquad.readAllEncoderData(encoderDataBlock);
        currentTicks = encoderDataBlock.positions[armEncoderChannel];
        currentVelocity = encoderDataBlock.velocities[armEncoderChannel];
        currentAngleDegrees = calculateCurrentArmAngleInDegrees();

        switch (currentState) {
            case ROTATING_CCW_TO_TARGET_ANGLE:
                double elapsedTimeMilliseconds = flipArmTimer.milliseconds();

                // Ramp-up logic
                double rampedPower = Math.min(elapsedTimeMilliseconds / SPECIMEN_ARM_PARAMS.RAMP_UP_TIME_MILLISECONDS , 1.0) * SPECIMEN_ARM_PARAMS.MAX_POWER;

                // Apply ramped power until reaching maximum or target angle
                if (elapsedTimeMilliseconds < SPECIMEN_ARM_PARAMS.RAMP_UP_TIME_MILLISECONDS) {
                    arm.setPower(rampedPower);
                } else if (rampedPower < SPECIMEN_ARM_PARAMS.MAX_POWER) {
                    arm.setPower(SPECIMEN_ARM_PARAMS.MAX_POWER); // Ensure constant power after ramp-up
                }

                //Reverse the intake at a certain time to let go of the game piece
                if ( elapsedTimeMilliseconds > SPECIMEN_ARM_PARAMS.REVERSE_FLIP_TIME_MS &&
                        Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE) &&
                        Robot.getInstance().getSpecimenIntakeSubsystem().isNotReversing()) {
                    Robot.getInstance().getSpecimenIntakeSubsystem().reverseIntake();
                }

                // Check if the arm has reached the target angle or timeout
                if (currentAngleDegrees >= targetAngleDegrees || flipArmTimer.seconds() > SPECIMEN_ARM_PARAMS.TIMEOUT_TIME_SECONDS) {
                    arm.setPower(0); // Stop the arm
                    setCurrentState(SpecimenArmStates.ZERO_POWER_AT_CCW_ARM_HOME);
                    zeroPowerTimer.reset();
                }
                break;

            case FLIPPING_TO_CW:
                if (flipArmTimer.milliseconds() > SPECIMEN_ARM_PARAMS.CW_FLIP_TIME_MS) {
                    arm.setPower(0);
                    setCurrentState(SpecimenArmStates.ZERO_POWER_AT_CW_ARM_HOME);
                    zeroPowerTimer.reset();
                    if (Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector().haveSpecimen()){
                        Robot.getInstance().getLightingSubsystem().setAllianceColor();
                    }
                }
                break;

            case FLIPPING_TO_CCW:
                double elapsedTime = flipArmTimer.milliseconds();
                if (elapsedTime > SPECIMEN_ARM_PARAMS.REVERSE_FLIP_TIME_MS &&
                        Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE) &&
                        Robot.getInstance().getSpecimenIntakeSubsystem().isNotReversing()) {
                    Robot.getInstance().getSpecimenIntakeSubsystem().reverseIntake();
                }
                if (elapsedTime > SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS) {
                    arm.setPower(0);
                    setCurrentState(SpecimenArmStates.ZERO_POWER_AT_CCW_ARM_HOME);
                    zeroPowerTimer.reset();
                    if (!Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector().haveSpecimen()){
                        Robot.getInstance().getLightingSubsystem().setLightBlack();
                    } else    Robot.getInstance().getLightingSubsystem().setProblemColor();
                }
                break;

            case ZERO_POWER_AT_CCW_ARM_HOME:
                if (zeroPowerTimer.milliseconds() > SPECIMEN_ARM_PARAMS.ZERO_POWER_SETTLE_TIME_MS) {
                    setCurrentState(SpecimenArmStates.CCW_ARM_HOME);
                    Robot.getInstance().getSpecimenIntakeSubsystem().setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                }
                break;

            case ZERO_POWER_AT_CW_ARM_HOME:
                if (zeroPowerTimer.milliseconds() > SPECIMEN_ARM_PARAMS.ZERO_POWER_SETTLE_TIME_MS) {
                    setCurrentState(SpecimenArmStates.CW_ARM_HOME);
                }
                break;

            case CCW_ARM_HOME:
            case CW_ARM_HOME:
            case SPECIMEN_PICKUP:
            default:
                // Check if movement is needed based on the target angle
                if (Math.abs(targetAngleDegrees - currentAngleDegrees) > SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES) {
                    moveToTargetAngle(); // Move towards the target if outside tolerance
                } else {
                    maintainPosition(); // Hold the position if within tolerance
                }
                break;
        }

        updateParameters();
        updateDashboardTelemetry();
        FlightRecorder.write("SPECIMEN_ARM_STATE", new SpecimenArmStateMessage(currentAngleDegrees, currentVelocity, currentState));
        FlightRecorder.write("SPECIMEN_ARM_POWER", new SpecimenArmPowerMessage(pidPower , feedforwardPower , clippedPower));
    }

    public void flipCCWFast() {
        flipArmTimer.reset();
        currentState = SpecimenArmStates.FLIPPING_TO_CCW;
        targetAngleDegrees = CCW_ARM_HOME.getArmAngle();
        pidController.setSetPoint(targetAngleDegrees);
        arm.setPower(SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CCW_FLIP);
    }

    public void flipCWFast() {
        flipArmTimer.reset();
        currentState = SpecimenArmStates.FLIPPING_TO_CW;
        targetAngleDegrees = CW_ARM_HOME.getArmAngle();
        pidController.setSetPoint(targetAngleDegrees);
        arm.setPower(SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CW_FLIP);
    }

    public void gotoPickupAngle() {
        setCurrentState(SpecimenArmStates.SPECIMEN_PICKUP);
    }

    public void increasePickupAngle() {
        MatchConfig.telemetryPacket.put("increase", currentState);
        if (currentState == SPECIMEN_PICKUP) {
            // Check if the adjustment is within the allowed range
            if (SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE < SPECIMEN_ARM_PARAMS.DEFAULT_PICKUP_ANGLE + SPECIMEN_ARM_PARAMS.MAX_PICKUP_ANGLE_ADJUSTMENT) {
                SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE += .5;
                setCurrentState(SPECIMEN_PICKUP);

            }
        }
    }

    public void decreasePickupAngle() {
        MatchConfig.telemetryPacket.put("decrease", currentState);

        if (currentState == SPECIMEN_PICKUP) {
            // Check if the adjustment is within the allowed range
            if (SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE > SPECIMEN_ARM_PARAMS.DEFAULT_PICKUP_ANGLE - SPECIMEN_ARM_PARAMS.MAX_PICKUP_ANGLE_ADJUSTMENT) {
                SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE -= .5;
                setCurrentState(SPECIMEN_PICKUP);
            }
        }
    }



    public void setManualTargetAngle(double armInput) {
        // Calculate the change in angle based on input and scale factor
        double deltaAngle = armInput * SPECIMEN_ARM_PARAMS.STICK_SCALE_FACTOR;

        // Calculate the new target angle based on the target angle
        targetAngleDegrees += deltaAngle;

        // Clip target angle to within allowed range
        targetAngleDegrees = Range.clip(targetAngleDegrees , SPECIMEN_ARM_PARAMS.CW_HOME , SPECIMEN_ARM_PARAMS.CCW_HOME);

        // Update the PID controller's setPoint with the new target angle
        pidController.setSetPoint(targetAngleDegrees);
    }

    public void setTargetAngle(SpecimenArmStates state) {
        currentState = state;
        targetAngleDegrees = state.getArmAngle();
        pidController.setSetPoint(targetAngleDegrees);
    }

    private void moveToTargetAngle() {
        // Calculate PID output
        pidPower = pidController.calculate(currentAngleDegrees);

        // Calculate feedforward output
        feedforwardPower = armFeedforward.calculate(
                Math.toRadians(currentAngleDegrees) ,
                0 ,
                0
        );

        // Combine PID and feedforward control efforts
        totalPower = pidPower + feedforwardPower;

        // Clip the total power to allowable range
        clippedPower = Range.clip(totalPower , - SPECIMEN_ARM_PARAMS.MAX_POWER , SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Apply motor power
        arm.setPower(clippedPower);
    }

    private void maintainPosition() {
        // PID control for position
        pidPower = pidController.calculate(currentAngleDegrees); // Pass current position, not error
        feedforwardPower = armFeedforward.calculate(
                Math.toRadians(currentAngleDegrees) ,
                0 ,
                0
        );
        totalPower = feedforwardPower + pidPower;
        clippedPower = Range.clip(totalPower , - SPECIMEN_ARM_PARAMS.MAX_POWER , SPECIMEN_ARM_PARAMS.MAX_POWER);
        arm.setPower(clippedPower);
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
                    SPECIMEN_ARM_PARAMS.kS * voltageScale ,
                    SPECIMEN_ARM_PARAMS.kCos ,
                    SPECIMEN_ARM_PARAMS.kV * voltageScale ,
                    SPECIMEN_ARM_PARAMS.kA * voltageScale
            );

            // Update the previous values to the current ones
            prev_kS = SPECIMEN_ARM_PARAMS.kS;
            prev_kCos = SPECIMEN_ARM_PARAMS.kCos;
            prev_kV = SPECIMEN_ARM_PARAMS.kV;
            prev_kA = SPECIMEN_ARM_PARAMS.kA;
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

    public void rotateToAngleWithRampingPower(double targetAngle) {
        arm.setPower(0); // Ensure arm starts with zero power
        targetAngleDegrees = targetAngle;
        flipArmTimer.reset(); // Start the timeout timer
        setCurrentState(SpecimenArmStates.ROTATING_CCW_TO_TARGET_ANGLE);
    }


    public void rotateToCCWWithConstantPower() {
        rotateToAngleWithRampingPower(SPECIMEN_ARM_PARAMS.CCW_FLIP_ARM_TARGET_ANGLE);
    }

    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("%s | Angle: %.2f" , currentState , currentAngleDegrees);
        telemetry.addLine(telemetryData);
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("specimenArm/Current State" , currentState);
    }

    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        // Display state and target state information on one line
        String stateOverview = String.format("Specimen Arm State: %s" , currentState);
        MatchConfig.telemetryPacket.addLine(stateOverview);

        // Add power overview on its own line
        String powerSummary = String.format("PID: %.2f | FF: %.2f | Clipped: %.2f" , pidPower , feedforwardPower , clippedPower);
        MatchConfig.telemetryPacket.addLine(powerSummary);

        MatchConfig.telemetryPacket.put("SpecimenArm/angles/Current Arm Angle" , String.format("%.2f" , currentAngleDegrees));
        MatchConfig.telemetryPacket.put("SpecimenArm/angles/Target Arm Angle" , String.format("%.2f" , targetAngleDegrees));

        MatchConfig.telemetryPacket.put("SpecimenArm/Current Arm Velocity" , String.format("%.2f" , currentVelocity));
    }
}


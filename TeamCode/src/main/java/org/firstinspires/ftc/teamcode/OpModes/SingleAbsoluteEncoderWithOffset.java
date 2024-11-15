package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Single Absolute Encoder with Offset", group="OctoQuad")
public class SingleAbsoluteEncoderWithOffset extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to the OctoQuad encoder module.
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Set the specific channel for the absolute encoder
        int absoluteChannel = 4;  // Change this to your desired channel (4-7 for pulse-width inputs)

        // Define the offset and direction multiplier for angle adjustment
        double angleOffset = 0.0;  // Calibrated zero offset for your encoder
        double directionMultiplier = 1.0;  // Set to -1.0 if reversing the angle is needed

        // Reset and configure the OctoQuad for pulse-width (absolute) encoding on the specified channel
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);
        octoquad.setSingleChannelPulseWidthParams(absoluteChannel, new OctoQuad.ChannelPulseWidthParams(1, 1024));

        // Conversion factor for microseconds to degrees (assuming 1024 us represents 360 degrees)
        final double DEGREES_PER_US = 360.0 / 1024.0;

        // Display OctoQuad firmware info and initial setup message
        telemetry.addLine("OctoQuad Initialized for Single Absolute Encoder");
        telemetry.addData("Firmware Version", octoquad.getFirmwareVersion());
        telemetry.update();

        // Wait for the OpMode to start
        waitForStart();

        // Timer for loop timing
        ElapsedTime elapsedTime = new ElapsedTime();
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();

        while (opModeIsActive()) {
            // Read encoder data for all channels and focus on the specified channel
            octoquad.readAllEncoderData(encoderDataBlock);

            // Get the raw position in microseconds and velocity for the chosen channel
            double rawPosition = encoderDataBlock.positions[absoluteChannel];
            double rawVelocity = encoderDataBlock.velocities[absoluteChannel];

            // Apply offset and direction adjustment to get a corrected angle in degrees
            double adjustedPositionDegrees = AngleUnit.normalizeDegrees((rawPosition * DEGREES_PER_US * directionMultiplier) - angleOffset);

            // Convert velocity to degrees per second, applying direction adjustment
            double adjustedVelocityDegreesPerSec = rawVelocity * DEGREES_PER_US * directionMultiplier;

            // Display telemetry for the specified channel
            telemetry.addData("Channel " + absoluteChannel + " Position (uS)", "%.2f", rawPosition);
            telemetry.addData("Adjusted Position (Degrees)", "%.2f", adjustedPositionDegrees);
            telemetry.addData("Velocity (uS/interval)", "%.2f", rawVelocity);
            telemetry.addData("Adjusted Velocity (deg/s)", "%.2f", adjustedVelocityDegreesPerSec);
            telemetry.addData("Loop time (ms)", "%.1f", elapsedTime.milliseconds());
            telemetry.update();

            // Reset the timer for the next loop
            elapsedTime.reset();
        }
    }
}

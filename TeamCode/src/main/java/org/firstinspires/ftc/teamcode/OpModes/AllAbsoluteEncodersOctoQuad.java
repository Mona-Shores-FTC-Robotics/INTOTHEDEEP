package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="All Absolute Encoders OctoQuad", group="OctoQuad")
public class AllAbsoluteEncodersOctoQuad extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to the OctoQuad by looking up its name in the hardwareMap.
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Configure the OctoQuad to use absolute encoders on all channels
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_PULSE_WIDTH);

        // Set up pulse width parameters for each channel (adjust range as needed for your encoder specs)
        for (int channel = 0; channel < 8; channel++) {
            octoquad.setSingleChannelPulseWidthParams(channel, new OctoQuad.ChannelPulseWidthParams(1, 1024));
        }

        // Constants for position conversion
        final double DEGREES_PER_US = 360.0 / 1044.0;

        // Display OctoQuad firmware information
        telemetry.addLine("OctoQuad Initialized with All Absolute Encoders");
        telemetry.addData("Firmware Version", octoquad.getFirmwareVersion());
        telemetry.update();

        waitForStart();

        // Timer for tracking loop performance
        ElapsedTime elapsedTime = new ElapsedTime();
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();

        while (opModeIsActive()) {
            // Read all encoder data into the encoderDataBlock
            octoquad.readAllEncoderData(encoderDataBlock);

            // Loop through each channel and display its position and velocity
            for (int channel = 0; channel < 8; channel++) {
                double position = encoderDataBlock.positions[channel];
                double degrees = position * DEGREES_PER_US;
                double velocity = encoderDataBlock.velocities[channel] * DEGREES_PER_US;

                // Display telemetry for each channel
                telemetry.addData("Channel " + channel + " Position (uS)", "%.2f", position);
                telemetry.addData("Channel " + channel + " Degrees", "%.2f", degrees);
                telemetry.addData("Channel " + channel + " Velocity (deg/s)", "%.2f", velocity);
            }

            telemetry.addData("Loop time (ms)", "%.1f", elapsedTime.milliseconds());
            telemetry.update();

            // Reset the timer for the next loop iteration
            elapsedTime.reset();
        }
    }
}

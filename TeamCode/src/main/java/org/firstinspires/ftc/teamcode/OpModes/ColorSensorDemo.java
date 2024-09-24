package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor Demo", group = "Sensor")
public class ColorSensorDemo extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Read the normalized RGB values
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to hue, saturation, and value
            float[] hsvValues = new float[3];
            android.graphics.Color.colorToHSV(colors.toColor(), hsvValues);

            // Determine the dominant color
            String dominantColor = getDominantColor(colors);

            // Display the color values
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Hue", "%.3f", hsvValues[0]);
            telemetry.addData("Dominant Color", dominantColor);

            // Perform actions based on the detected color
            switch (dominantColor) {
                case "Red":
                    telemetry.addLine("Red detected: Perform red action");
                    // Add your red action here
                    break;
                case "Green":
                    telemetry.addLine("Green detected: Perform green action");
                    // Add your green action here
                    break;
                case "Blue":
                    telemetry.addLine("Blue detected: Perform blue action");
                    // Add your blue action here
                    break;
                default:
                    telemetry.addLine("No dominant color detected");
            }

            telemetry.update();
        }
    }

    private String getDominantColor(NormalizedRGBA colors) {
        if (colors.red > colors.green && colors.red > colors.blue) {
            return "Red";
        } else if (colors.green > colors.red && colors.green > colors.blue) {
            return "Green";
        } else if (colors.blue > colors.red && colors.blue > colors.green) {
            return "Blue";
        } else {
            return "Unknown";
        }
    }
}
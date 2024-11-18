package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.arcrobotics.ftclib.controller.PIDFController;

public class AnglePIDController extends PIDFController {

    public AnglePIDController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, kf);
    }

    @Override
    public double calculate(double currentAngle, double targetAngle) {
        // Wrap the error to [-PI, PI]
        double error = wrapAngle(targetAngle - currentAngle);

        // Set the wrapped error internally as the position error for PID calculations
        setSetPoint(error);

        // Calculate the output using the wrapped error
        return super.calculate(0); // We pass 0 because we already calculated the error
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

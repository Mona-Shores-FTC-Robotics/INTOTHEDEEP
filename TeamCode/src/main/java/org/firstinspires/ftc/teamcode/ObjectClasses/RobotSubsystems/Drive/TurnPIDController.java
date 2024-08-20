package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class TurnPIDController {
    private double m_lastDegreesLeftToTurn;
    double integralSum = 0;
    private double targetAngle;
    double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;

    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double feedforward;

    public double error;

    public TurnPIDController(double target, double p, double i, double d, double f) {
        targetAngle = target;
        Kp = p;
        Ki = i;
        Kd = d;
        feedforward = f;
    }

    public double update(double currentAngle) {
        //P Term
        error = targetAngle - currentAngle;
        //The java modulo operator can return negative values, so this will make the error between -360 to 360
        error %= 360;

        //These two operations in combination make sure the error is between 0-359
        error += 360;
        error %= 360;

        //This check gives us the shortest path to turn by giving us values 0 to 180 or 0 to -180
        if (error > 180) error -= 360;

        //I Term
        accumulatedError += error * timer.seconds();
        if (Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        //D Term
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;
        timer.reset();

        //motor power calculation
        double output = feedforward * Math.signum(error) + .9 * Math.tanh(
                (error * Kp) + (accumulatedError * Ki) + (slope * Kd));

        Robot.getInstance().getActiveOpMode().telemetry.addData("Turn PID error", error);

        return -output;
    }
}






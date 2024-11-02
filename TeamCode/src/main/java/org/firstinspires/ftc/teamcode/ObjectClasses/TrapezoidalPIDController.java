package org.firstinspires.ftc.teamcode.ObjectClasses;

public class TrapezoidalPIDController {

    private double maxVelocity;
    private double accelerationRate;
    private double currentVelocity;
    private double maxPower;

    // PID gains
    private double kP;
    private double kI;
    private double kD;

    // Variables for telemetry/debugging
    private double profilePower;
    private double pidAdjustment;
    private double feedforwardPower;

    // Movement direction
    public enum MovementDirection {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    private MovementDirection movementDirection;

    // PID internal variables
    private double integral = 0;
    private double previousError = 0;
    private double currentError = 0;

    // Setpoint and tolerance
    private double setpoint = 0;
    private double positionTolerance = 1.0; // Default tolerance in degrees

    public TrapezoidalPIDController(double p, double i, double d, double maxVelocity, double accelerationRate, double maxPower) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.maxVelocity = maxVelocity;
        this.accelerationRate = accelerationRate;
        this.currentVelocity = 0;
        this.maxPower = maxPower;
        this.movementDirection = null; // Must be set before use
    }

    // Getter and Setter methods for PID gains
    public double getP() {
        return kP;
    }

    public void setP(double p) {
        this.kP = p;
    }

    public double getI() {
        return kI;
    }

    public void setI(double i) {
        this.kI = i;
    }

    public double getD() {
        return kD;
    }

    public void setD(double d) {
        this.kD = d;
    }

    // Getter and Setter for Setpoint
    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = normalizeAngle(setpoint);
    }

    public void setMovementDirection(MovementDirection direction) {
        this.movementDirection = direction;
    }

    public void setTolerance(double tolerance) {
        this.positionTolerance = Math.abs(tolerance);
    }

    public boolean atSetpoint() {
        return Math.abs(currentError) <= positionTolerance;
    }

    public double calculatePowerWithProfile(double currentAngle, double feedforwardPower) {
        // Ensure movement direction is set
        if (movementDirection == null) {
            throw new IllegalStateException("Movement direction must be set before calling calculatePowerWithProfile.");
        }

        // Normalize current angle
        currentAngle = normalizeAngle(currentAngle);

        // Use stored setpoint
        double targetAngle = setpoint;

        // Calculate error based on movement direction
        if (movementDirection == MovementDirection.COUNTERCLOCKWISE) {
            currentError = -(targetAngle - currentAngle + 360) % 360;
        } else if (movementDirection == MovementDirection.CLOCKWISE) {
            currentError = ((currentAngle - targetAngle + 360) % 360);
        }

        // PID calculations
        integral += currentError;
        double derivative = currentError - previousError;
        previousError = currentError;

        this.pidAdjustment = kP * currentError + kI * integral + kD * derivative;

        // Apply trapezoidal motion profiling logic as needed
        this.profilePower = calculateProfilePower(currentError);

        // Store the feedforward power for telemetry
        this.feedforwardPower = feedforwardPower;

        // Combine profile power, PID output, and feedforward power
        double combinedPower = profilePower + pidAdjustment + feedforwardPower;

        // Clip the combined power to the max allowed power
        return clip(combinedPower, -maxPower, maxPower);
    }

    private double calculateProfilePower(double error) {
        // Implement your trapezoidal motion profiling logic here
        // For now, return 0 or implement as needed
        return 0; // Placeholder
    }

    private double normalizeAngle(double angle) {
        return (angle % 360 + 360) % 360; // Normalize to 0–360 degrees
    }

    // Custom clipping method if Range.clip() isn't available
    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    // Getters for telemetry
    public double getProfilePower() {
        return profilePower;
    }

    public double getPidAdjustment() {
        return pidAdjustment;
    }

    public double getFeedforwardPower() {
        return feedforwardPower;
    }

    // Method to reset the controller (optional)
    public void reset() {
        integral = 0;
        previousError = 0;
        currentError = 0;
    }
}

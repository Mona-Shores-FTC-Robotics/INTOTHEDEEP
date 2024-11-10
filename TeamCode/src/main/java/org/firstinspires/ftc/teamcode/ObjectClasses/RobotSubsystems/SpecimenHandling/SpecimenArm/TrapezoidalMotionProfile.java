package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm;

public class TrapezoidalMotionProfile {
    private final double maxAcceleration; // Acceleration in degrees per millisecond^2
    private final double maxVelocity; // Velocity in degrees per millisecond
    private final double distanceInDegrees;

    private double accelerationTimeMilliseconds;
    private double cruiseTimeMilliseconds;
    private double totalTimeMilliseconds;

    public TrapezoidalMotionProfile(double maxAcceleration, double maxVelocity, double distance) {
        this.maxAcceleration = maxAcceleration / 1000; // Convert to degrees per millisecond^2
        this.maxVelocity = maxVelocity / 1000; // Convert to degrees per millisecond
        this.distanceInDegrees = distance; // Convert to degrees
        calculateProfile();
    }

    private void calculateProfile() {
        double accelTime = maxVelocity / maxAcceleration;
        double accelDistance = 0.5 * maxAcceleration * Math.pow(accelTime, 2);

        if (accelDistance * 2 > distanceInDegrees) {
            // Triangular profile
            accelerationTimeMilliseconds = Math.sqrt(distanceInDegrees / maxAcceleration);
            cruiseTimeMilliseconds = 0;
            totalTimeMilliseconds = 2 * accelerationTimeMilliseconds;
        } else {
            // Trapezoidal profile
            accelerationTimeMilliseconds = accelTime;
            double cruiseDistance = distanceInDegrees - 2 * accelDistance;
            cruiseTimeMilliseconds = cruiseDistance / maxVelocity;
            totalTimeMilliseconds = 2 * accelerationTimeMilliseconds + cruiseTimeMilliseconds;
        }
    }

    public double getTotalTimeMilliseconds() {
        return totalTimeMilliseconds;
    }

    public double getMotionProfileAngleInDegrees(double currentTimeMilliseconds) {
        if (currentTimeMilliseconds >= totalTimeMilliseconds) {
            return distanceInDegrees;
        }

        if (currentTimeMilliseconds < accelerationTimeMilliseconds) {
            // Acceleration phase
            return 0.5 * maxAcceleration * Math.pow(currentTimeMilliseconds, 2);
        } else if (currentTimeMilliseconds < (accelerationTimeMilliseconds + cruiseTimeMilliseconds)) {
            // Cruise phase
            double accelDistance = 0.5 * maxAcceleration * Math.pow(accelerationTimeMilliseconds, 2);
            double cruiseTimeElapsed = currentTimeMilliseconds - accelerationTimeMilliseconds;
            return accelDistance + maxVelocity * cruiseTimeElapsed;
        } else {
            // Deceleration phase
            double accelDistance = 0.5 * maxAcceleration * Math.pow(accelerationTimeMilliseconds, 2);
            double cruiseDistance = maxVelocity * cruiseTimeMilliseconds;
            double decelTimeElapsed = currentTimeMilliseconds - accelerationTimeMilliseconds - cruiseTimeMilliseconds;
            return accelDistance + cruiseDistance + maxVelocity * decelTimeElapsed - 0.5 * maxAcceleration * Math.pow(decelTimeElapsed, 2);
        }
    }

    public double getVelocity(double currentTime) {
        if (currentTime >= totalTimeMilliseconds) {
            return 0;
        }

        if (currentTime < accelerationTimeMilliseconds) {
            // Acceleration phase
            return maxAcceleration * currentTime;
        } else if (currentTime < (accelerationTimeMilliseconds + cruiseTimeMilliseconds)) {
            // Cruise phase
            return maxVelocity;
        } else {
            // Deceleration phase
            double decelTimeElapsed = currentTime - accelerationTimeMilliseconds - cruiseTimeMilliseconds;
            return maxVelocity - maxAcceleration * decelTimeElapsed;
        }
    }

    public double getAcceleration(double currentTime) {
        if (currentTime >= totalTimeMilliseconds) {
            return 0; // No acceleration after the profile ends
        }

        if (currentTime < accelerationTimeMilliseconds) {
            // Acceleration phase
            return maxAcceleration;
        } else if (currentTime < (accelerationTimeMilliseconds + cruiseTimeMilliseconds)) {
            // Cruise phase
            return 0;
        } else {
            // Deceleration phase
            return -maxAcceleration;
        }
    }

    public boolean isFinished(double currentTime) {
        return currentTime >= totalTimeMilliseconds;
    }
}

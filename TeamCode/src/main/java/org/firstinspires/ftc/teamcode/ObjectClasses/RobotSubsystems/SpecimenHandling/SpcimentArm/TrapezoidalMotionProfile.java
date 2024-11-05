package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

public class TrapezoidalMotionProfile {
    private final double maxAcceleration;
    private final double maxVelocity;
    private final double distance;

    private double accelerationTime;
    private double cruiseTime;
    private double totalTime;

    public TrapezoidalMotionProfile(double maxAcceleration, double maxVelocity, double distance) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.distance = distance;

        calculateProfile();
    }

    private void calculateProfile() {
        double accelTime = maxVelocity / maxAcceleration;
        double accelDistance = 0.5 * maxAcceleration * Math.pow(accelTime, 2);

        if (accelDistance * 2 > distance) {
            // Triangular profile
            accelerationTime = Math.sqrt(distance / maxAcceleration);
            cruiseTime = 0;
            totalTime = 2 * accelerationTime;
        } else {
            // Trapezoidal profile
            accelerationTime = accelTime;
            double cruiseDistance = distance - 2 * accelDistance;
            cruiseTime = cruiseDistance / maxVelocity;
            totalTime = 2 * accelerationTime + cruiseTime;
        }
    }

    /**
     * Returns the position at the given time.
     *
     * @param currentTime The elapsed time since the start of the motion profile.
     * @return The position in degrees.
     */
    public double getPosition(double currentTime) {
        if (currentTime >= totalTime) {
            return distance;
        }

        if (currentTime < accelerationTime) {
            // Acceleration phase
            return 0.5 * maxAcceleration * Math.pow(currentTime, 2);
        } else if (currentTime < (accelerationTime + cruiseTime)) {
            // Cruise phase
            double accelDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
            double cruiseTimeElapsed = currentTime - accelerationTime;
            return accelDistance + maxVelocity * cruiseTimeElapsed;
        } else {
            // Deceleration phase
            double accelDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
            double cruiseDistance = maxVelocity * cruiseTime;
            double decelTimeElapsed = currentTime - accelerationTime - cruiseTime;
            return accelDistance + cruiseDistance + maxVelocity * decelTimeElapsed - 0.5 * maxAcceleration * Math.pow(decelTimeElapsed, 2);
        }
    }

    /**
     * Returns the velocity at the given time.
     *
     * @param currentTime The elapsed time since the start of the motion profile.
     * @return The velocity in degrees per second.
     */
    public double getVelocity(double currentTime) {
        if (currentTime >= totalTime) {
            return 0;
        }

        if (currentTime < accelerationTime) {
            // Acceleration phase
            return maxAcceleration * currentTime;
        } else if (currentTime < (accelerationTime + cruiseTime)) {
            // Cruise phase
            return maxVelocity;
        } else {
            // Deceleration phase
            double decelTimeElapsed = currentTime - accelerationTime - cruiseTime;
            return maxVelocity - maxAcceleration * decelTimeElapsed;
        }
    }

    /**
     * Checks if the motion profile has finished.
     *
     * @param currentTime The elapsed time since the start of the motion profile.
     * @return True if the profile is finished, false otherwise.
     */
    public boolean isFinished(double currentTime) {
        return currentTime >= totalTime;
    }
}

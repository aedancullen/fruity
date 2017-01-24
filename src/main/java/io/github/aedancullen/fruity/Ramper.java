package io.github.aedancullen.fruity;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * Ramper.java
 * A 'movement smoother' that creates linear control curves in order
 * to gain smoother acceleration/deceleration from the gamepad controls.
 *
 * (c) 2016 Aedan Cullen. Distributed under the GNU GPLv3 license.
 */

public class Ramper {

    double targetValue;

    double currentValue;

    double rampUpRate;
    double rampDownRate;

    double deadZone;

    boolean rampDownEnabled;

    long lastRamp = 0;

    public Ramper(double rampUpRate, double rampDownRate, double deadZone, boolean rampDownEnabled) {
        this.rampUpRate = rampUpRate;
        this.rampDownRate = rampDownRate;
        this.rampDownEnabled = rampDownEnabled;
        this.deadZone = deadZone;
    }

    public void ramp(double value) {
        targetValue = value;

        if (lastRamp == 0) {
            lastRamp = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - lastRamp;
        lastRamp = System.currentTimeMillis();


        if (Math.abs(currentValue - targetValue) < deadZone) {
            currentValue = targetValue;
        }


            if (Math.abs(targetValue) < Math.abs(currentValue) && rampDownEnabled) {
                if (targetValue < currentValue) {
                    currentValue -= rampDownRate * elapsed;
                }
                else {
                    currentValue += rampDownRate * elapsed;
                }
            }
            else if (Math.abs(targetValue) < Math.abs(currentValue) && !rampDownEnabled) {
                currentValue = targetValue;
            }
            else if (Math.abs(targetValue) > Math.abs(currentValue)) {
                if (targetValue < currentValue) {
                    currentValue -= rampUpRate * elapsed;
                }
                else {
                    currentValue += rampUpRate * elapsed;
                }
            }

    }

    public double getValue() {
        return currentValue;
    }

}

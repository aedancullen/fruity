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

    boolean rampDownEnabled;

    long lastRamp = 0;

    public Ramper(double rampUpRate, double rampDownRate, boolean rampDownEnabled) {
        this.rampUpRate = rampUpRate;
        this.rampDownRate = rampDownRate;
        this.rampDownEnabled = rampDownEnabled;
    }

    public void ramp(double value) {
        targetValue = value;

        if (lastRamp == 0) {
            lastRamp = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - lastRamp;
        lastRamp = System.currentTimeMillis();

        if (!rampDownEnabled && targetValue - currentValue < 0) {
            currentValue = targetValue;
        }
        else {
            if (targetValue < currentValue) {
                currentValue -= rampDownRate * elapsed;
            }
            else if (targetValue > currentValue) {
                currentValue += rampUpRate * elapsed;
            }
        }
    }

    public double getValue() {
        return currentValue;
    }

}

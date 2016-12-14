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

    double targetTranslationPower;
    double targetRotationPower;

    double currentTranslationPower;
    double currentRotationPower;

    double translationPowerRampRate;
    double rotationPowerRampRate;

    boolean rampDownEnabled;

    long lastRamp = 0;

    public Ramper(double translationPowerRampRate, double rotationPowerRampRate, boolean rampDownEnabled) {
        this.translationPowerRampRate = translationPowerRampRate;
        this.rotationPowerRampRate = rotationPowerRampRate;
        this.rampDownEnabled = rampDownEnabled;
    }

    public void ramp(double translationPower, double rotationPower) {
        targetTranslationPower = translationPower;
        targetRotationPower = rotationPower;
        if (lastRamp == 0) {
            lastRamp = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - lastRamp;
        lastRamp = System.currentTimeMillis();

        if (!rampDownEnabled && targetTranslationPower - currentTranslationPower < 0) {
            currentTranslationPower = targetTranslationPower;
        }
        else {
            currentTranslationPower += ((targetTranslationPower - currentTranslationPower) * translationPowerRampRate * elapsed);
        }

        if (!rampDownEnabled && targetRotationPower - currentRotationPower < 0) {
            currentRotationPower = targetRotationPower;
        }
        else {
            currentRotationPower += ((targetRotationPower - currentRotationPower) * rotationPowerRampRate * elapsed);
        }
    }

    public double getTranslationPower() {
        return currentTranslationPower;
    }

    public double getRotationPower() {
        return currentRotationPower;
    }

}

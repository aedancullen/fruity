package io.github.aedancullen.fruity;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * Ramper.java
 * A 'movement smoother' that creates linear control in order
 * to gain smoother acceleration from the gamepad controls and smooth control of
 * rotation power in response to a difference between necessary and observed heading.
 *
 * (c) 2016 Aedan Cullen. Distributed under the GNU GPLv3 license.
 */

public class Ramper {

    double currentTranslationPower;
    double currentRotationPower;

    double translationPowerRampRate;
    double rotationPowerRampRate;

    long lastRamp = 0;

    public Ramper(double translationPowerRampRate, double rotationPowerRampRate) {
        this.translationPowerRampRate = translationPowerRampRate;
        this.rotationPowerRampRate = rotationPowerRampRate;
    }

    public void ramp(double targetTranslationPower, double targetAngle, double currentAngle) {
        if (lastRamp == 0) {
            lastRamp = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - lastRamp;
        lastRamp = System.currentTimeMillis();
        if (targetTranslationPower == 0) {
            currentTranslationPower = 0;
        }
        else {
            currentTranslationPower += ((targetTranslationPower - currentTranslationPower) * translationPowerRampRate * elapsed);
        }
        currentRotationPower = ((targetAngle - currentAngle) * rotationPowerRampRate);
    }

    public double getTranslationPower() {
        return currentTranslationPower;
    }

    public double getRotationPower() {
        return currentRotationPower;
    }

}

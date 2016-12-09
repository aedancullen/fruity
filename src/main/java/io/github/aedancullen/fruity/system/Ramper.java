package io.github.aedancullen.fruity.system;

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

    long lastRamp = 0;

    public Ramper(double translationPowerRampRate, double rotationPowerRampRate) {
        this.translationPowerRampRate = translationPowerRampRate;
        this.rotationPowerRampRate = rotationPowerRampRate;
    }

    public void ramp(double translationPower, double rotationPower) {
        targetTranslationPower = translationPower;
        targetRotationPower = rotationPower;
        if (lastRamp == 0) {
            lastRamp = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - lastRamp;
        lastRamp = System.currentTimeMillis();
        if (Math.abs(currentTranslationPower - targetTranslationPower) > 0.05) {
            if (currentTranslationPower > targetTranslationPower) {
                currentTranslationPower -= (translationPowerRampRate * elapsed);
            } else {
                currentTranslationPower += (translationPowerRampRate * elapsed);
            }
        }
        else {
            currentTranslationPower = targetTranslationPower;
        }
        if (Math.abs(currentRotationPower - targetRotationPower) > 0.05) {
            if (currentRotationPower > targetRotationPower) {
                currentRotationPower -= (rotationPowerRampRate * elapsed);
            } else {
                currentRotationPower += (rotationPowerRampRate * elapsed);
            }
        }
        else {
            currentRotationPower = targetRotationPower;
        }
    }

    public double getTranslationPower() {
        return currentTranslationPower;
    }

    public double getRotationPower() {
        return currentRotationPower;
    }

}

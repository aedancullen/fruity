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

    double targetAngle;
    double targetTranslationPower;
    double targetRotationPower;

    double currentAngle;
    double currentTranslationPower;
    double currentRotationPower;

    double angleRampRate;
    double translationPowerRampRate;
    double rotationPowerRampRate;

    long lastRamp;

    public Ramper(double angleRampRate, double translationPowerRampRate, double rotationPowerRampRate) {
        this.angleRampRate = angleRampRate;
        this.translationPowerRampRate = translationPowerRampRate;
        this.rotationPowerRampRate = rotationPowerRampRate;
    }

    public void ramp(EssentialHeading heading, double translationPower, double rotationPower) {
        targetAngle = heading.getAngleDegrees();
        targetTranslationPower = translationPower;
        targetRotationPower = rotationPower;
        if (lastRamp == 0) {
            lastRamp = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - lastRamp;
        lastRamp = System.currentTimeMillis();
        if (Math.abs(currentAngle - targetAngle) > 5) {
            if (currentAngle > targetAngle) {
                currentAngle -= (angleRampRate * elapsed);
            } else {
                currentAngle += (angleRampRate * elapsed);
            }
        }
        if (Math.abs(currentTranslationPower - targetTranslationPower) > 0.05) {
            if (currentTranslationPower > targetTranslationPower) {
                currentTranslationPower -= (translationPowerRampRate * elapsed);
            } else {
                currentTranslationPower += (translationPowerRampRate * elapsed);
            }
        }
        if (Math.abs(currentTranslationPower - targetTranslationPower) > 0.05) {
            if (currentTranslationPower > targetTranslationPower) {
                currentTranslationPower -= (translationPowerRampRate * elapsed);
            } else {
                currentTranslationPower += (translationPowerRampRate * elapsed);
            }
        }
        if (Math.abs(currentRotationPower - targetRotationPower) > 0.05) {
            if (currentRotationPower > targetRotationPower) {
                currentRotationPower -= (rotationPowerRampRate * elapsed);
            } else {
                currentRotationPower += (rotationPowerRampRate * elapsed);
            }
        }

    }

    public EssentialHeading getHeading() {
        return new EssentialHeading(currentAngle);
    }

    public double getTranslationPower() {
        return currentTranslationPower;
    }

    public double getRotationPower() {
        return currentRotationPower;
    }

}

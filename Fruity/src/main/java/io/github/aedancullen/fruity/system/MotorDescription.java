package io.github.cn04.fruity.system;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * MotorDescription.java
 * A structure to store information about a particular
 * set of parallel (facing in the same direction) wheels in an omnibot design.
 *
 * (c) 2016 Aedan Cullen. Distributed under the MIT License.
 */

public class MotorDescription {

    private EssentialHeading heading;
    private double rotationGain;

    public MotorDescription(int angleDegrees, double rotationGain) {
        if (rotationGain > 1 || rotationGain <= 0) {
            throw new IllegalArgumentException("Rotation gain must be less than or equal to 1, and greater than 0");
        }
        this.heading = new EssentialHeading(angleDegrees);
        this.rotationGain = rotationGain;
    }

    public EssentialHeading getEssentialHeading() {
        return heading;
    }

    public double getRotationGain() {
        return rotationGain;
    }

}

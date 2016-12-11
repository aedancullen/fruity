package io.github.aedancullen.fruity;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * EssentialHeading.java
 * A structure to store headings in a convenient, compact representation.
 *
 * (c) 2016 Aedan Cullen. Distributed under the GNU GPLv3 license.
 */

public class EssentialHeading {

    private double angleDegrees;

    public EssentialHeading(double angleDegrees) {
        if (angleDegrees <= -180) {
            this.angleDegrees = angleDegrees + 360;
        }
        else if (angleDegrees > 180) {
            this.angleDegrees = angleDegrees - 360;
        }
        else {
            this.angleDegrees = angleDegrees;
        }
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    public EssentialHeading regularizeToSemicircle() {
        if (angleDegrees < -90 || angleDegrees > 90) {
            return new EssentialHeading(180 - angleDegrees);
        }
        else {
            return new EssentialHeading(angleDegrees);
        }
    }

    public EssentialHeading subtract(EssentialHeading other) {
        return new EssentialHeading(this.angleDegrees - other.angleDegrees);
    }

    public static EssentialHeading fromOrientation(Orientation orientation) {
        return new EssentialHeading(orientation.firstAngle);
    }

    public static EssentialHeading fromInvertedOrientation(Orientation invertedOrientation) {
        return new EssentialHeading(-invertedOrientation.firstAngle);
    }

}

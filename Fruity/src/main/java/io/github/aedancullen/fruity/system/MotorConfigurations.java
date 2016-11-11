package io.github.cn04.fruity.system;

import java.util.Arrays;
import java.util.List;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * MotorConfigurations.java
 * Defines several common formats of omnidirectional robot configurations.
 * Each can be passed to a FruityController along with a parallel list of motors for use.
 *
 * (c) 2016 Aedan Cullen. Distributed under the MIT License.
 */

public class MotorConfigurations {

    public static List<MotorDescription> QUAD_NONDIAGONAL = Arrays.asList(
            new MotorDescription(0, 1),
            new MotorDescription(90, 1),
            new MotorDescription(180, 1),
            new MotorDescription(270, 1)
    );

    public static List<MotorDescription> QUAD_NONDIAGONAL_SHORT = Arrays.asList(
            new MotorDescription(0, 0.7),
            new MotorDescription(90, 1),
            new MotorDescription(180, 0.7),
            new MotorDescription(270, 1)
    );

    public static List<MotorDescription> QUAD_DIAGONAL = Arrays.asList(
            new MotorDescription(45, 1),
            new MotorDescription(135, 1),
            new MotorDescription(225, 1),
            new MotorDescription(315, 1)
    );

    public static List<MotorDescription> KIWI = Arrays.asList(
            new MotorDescription(60, 1),
            new MotorDescription(120, 1),
            new MotorDescription(270, 1)
    );

}

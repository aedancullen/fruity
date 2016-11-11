package io.github.cn04.fruity.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

import io.github.cn04.fruity.system.FruityController;
import io.github.cn04.fruity.system.MotorConfigurations;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * FruityGamepadDriving.java
 * A fun demo of omnidirectional driving capability, made simple!
 *
 * (c) 2016 Aedan Cullen. Distributed under the MIT License.
 */

@TeleOp(name="cn04.fruity: Gamepad Driving Demo", group="Fruity")
//@Disabled

public class FruityGamepadDriving extends OpMode {

    FruityController fruity;

    public void init() {
        fruity = new FruityController(hardwareMap, telemetry, "",
                Arrays.asList(
                        hardwareMap.dcMotor.get("dcOmni0"),
                        hardwareMap.dcMotor.get("dcOmni90"),
                        hardwareMap.dcMotor.get("dcOmni180"),
                        hardwareMap.dcMotor.get("dcOmni270")
                ),
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                MotorConfigurations.QUAD_NONDIAGONAL_SHORT);
    }

    public void loop() {
        fruity.handleGamepad(gamepad1);
    }

}

package io.github.aedancullen.fruity;

import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

import static java.lang.Double.NaN;

/**
 * ----- Fruity Omnidirectional Control System for FTC -----
 *
 * FruityController.java
 * A general control algorithm for any type of omnidirectional driving design.
 * Accepts motor arrangements in the form of Lists of DcMotors and MotorDescriptions.
 *
 * (c) 2016 Aedan Cullen. Distributed under the GNU GPLv3 license.
 */

public class FruityController {

    public static String TAG = "Fruity Controller";

    private List<DcMotor> motors;
    private List<MotorDescription> motorConfiguration;
    HardwareMap hardwareMap;
    private Telemetry telemetry;
    BNO055IMU imu;
    EssentialHeading headingStraight;
    Ramper ramper;
    boolean usingRamper = false;

    double lastStickAngle = NaN;

    long lastTime;

    boolean wasPressedLastTime = false;

    private int[] movedDistanceZero;

    EssentialHeading holdingHeading = new EssentialHeading(0);

    public FruityController(HardwareMap hardwareMap,
                            Telemetry telemetry,
                            String imuName,
                            List<DcMotor> motors,
                            DcMotorSimple.Direction clockwiseDirection,
                            DcMotor.RunMode runMode,
                            List<MotorDescription> motorConfiguration
    ) {
        Log.i(TAG, "[CONSTRUCTOR] Starting up...");
        if (motors.size() != motorConfiguration.size()) {
            throw new IllegalArgumentException("The same number of motors must be supplied as the number of motor descriptions");
        }
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.motors = motors;
        this.motorConfiguration = motorConfiguration;
        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            motor.setMode(runMode);
            motor.setDirection(clockwiseDirection);
        }
        movedDistanceZero = new int[motors.size()];
        headingStraight = new EssentialHeading(0);
        String imuStatus;
        if (!imuName.equals("")) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            imu = hardwareMap.get(BNO055IMU.class, imuName);
            imu.initialize(parameters);
            Orientation orientationStraight = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            headingStraight = EssentialHeading.fromInvertedOrientation(orientationStraight);
            Log.i(TAG, "[CONSTRUCTOR] IMU enabled!");
            imuStatus = imuName;
        }
        else {
            Log.i(TAG, "[CONSTRUCTOR] Not using an IMU");
            imuStatus = "not in use";
        }
        Log.i(TAG, "[CONSTRUCTOR] Straight heading: " + headingStraight.getAngleDegrees());
        telemetry.addData("* Fruity Controller", "Initialized " + motors.size() + " motors. IMU: " + imuStatus);
    }

    public void setupRamper(double rampUpRate, double rampDownRate, double deadZone, boolean rampDownEnabled) {
        ramper = new Ramper(rampUpRate, rampDownRate, deadZone, rampDownEnabled);
        usingRamper = true;
    }

    public void setupRamper(double rampUpRate, double rampDownRate, boolean rampDownEnabled) {
        ramper = new Ramper(rampUpRate, rampDownRate, 0.05, rampDownEnabled);
        usingRamper = true;
    }

    public void disableRamper() {
        usingRamper = false;
    }

    public void handleGamepad(Gamepad gamepad, double angleSnapGain) {
        if (gamepad.back) {
            Orientation orientationStraight = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            headingStraight = EssentialHeading.fromInvertedOrientation(orientationStraight);
            holdingHeading = new EssentialHeading(0);
        }

        double translationPower = Math.sqrt(Math.pow(gamepad.right_stick_x,2) + Math.pow(gamepad.right_stick_y,2)) / 2;

        double stickAngle = Math.toDegrees(Math.atan(gamepad.right_stick_x / -gamepad.right_stick_y));
        if (-gamepad.right_stick_y <= 0) { stickAngle = 180 + stickAngle; }

        if (translationPower == 0) {
            translationPower = 0.45;
            if (gamepad.y && !gamepad.x && !gamepad.b) {
                stickAngle = 0;
            } else if (gamepad.y && gamepad.b) {
                stickAngle = 45;
            } else if (gamepad.b && !gamepad.y && !gamepad.a) {
                stickAngle = 90;
            } else if (gamepad.b && gamepad.a) {
                stickAngle = 135;
            } else if (gamepad.a && !gamepad.b && !gamepad.x) {
                stickAngle = 180;
            } else if (gamepad.a && gamepad.x) {
                stickAngle = -135;
            } else if (gamepad.x && !gamepad.a && !gamepad.y) {
                stickAngle = -90;
            } else if (gamepad.x && gamepad.y) {
                stickAngle = -45;
            } else {
                translationPower = 0;
            }
        }


        if (Double.isNaN(stickAngle)) { // Joystick in center
            if (Double.isNaN(lastStickAngle)) { // last joystick reading does not exist yet
                stickAngle = 180; // Initialize to straight
                lastStickAngle = 180;
            }
            else {
                stickAngle = lastStickAngle; // Last joystick reading DOES EXIST, and joystick is currently in center. In order to keep ramping down correctly, keep old angle
            }
        }
        else {
            lastStickAngle = stickAngle; // IF there's a non-center joystick, just set the 'last' angle to the current and proceed
        }
        EssentialHeading stickHeading = new EssentialHeading(stickAngle);
        Log.d(TAG, "[GAMEPAD] Stick heading: " + stickHeading.getAngleDegrees());
        Log.d(TAG, "[GAMEPAD] Stick deflection (translation power): " + translationPower);

        //Are we angle-snapping?
        if (gamepad.dpad_up && !gamepad.dpad_right && !gamepad.dpad_left) {
            holdingHeading = new EssentialHeading(0);
        }
        else if (gamepad.dpad_up && gamepad.dpad_right) {
            holdingHeading = new EssentialHeading(45);
        }
        else if (gamepad.dpad_right && !gamepad.dpad_up && !gamepad.dpad_down) {
            holdingHeading = new EssentialHeading(90);
        }
        else if (gamepad.dpad_right && gamepad.dpad_down) {
            holdingHeading = new EssentialHeading(135);
        }
        else if (gamepad.dpad_down && !gamepad.dpad_right && !gamepad.dpad_left) {
            holdingHeading = new EssentialHeading(180);
        }
        else if (gamepad.dpad_down && gamepad.dpad_left) {
            holdingHeading = new EssentialHeading(-135);
        }
        else if (gamepad.dpad_left && !gamepad.dpad_down && !gamepad.dpad_up) {
            holdingHeading = new EssentialHeading(-90);
        }
        else if (gamepad.dpad_left && gamepad.dpad_up) {
            holdingHeading = new EssentialHeading(-45);
        }

        double rotationPower = this.getNecessaryRotationPower((new EssentialHeading(headingStraight.getAngleDegrees() + holdingHeading.getAngleDegrees())), angleSnapGain);

        if (gamepad.right_bumper) {
            rotationPower = 0.1;
            wasPressedLastTime = true;
        }
        else if (gamepad.left_bumper) {
            rotationPower = -0.1;
            wasPressedLastTime = true;
        }
        else if (gamepad.left_stick_x != 0) {
            rotationPower = gamepad.left_stick_x / 4;
            wasPressedLastTime = true;
        }
        else {
            if (wasPressedLastTime) {
                wasPressedLastTime = false;
                Orientation orientationNow = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                EssentialHeading headingNow = EssentialHeading.fromInvertedOrientation(orientationNow);
                holdingHeading = headingNow.subtract(headingStraight);
            }
        }

        Log.d(TAG, "[GAMEPAD] Rotation power:" + rotationPower);
        if (usingRamper) {
            driveWithRamper(stickHeading, translationPower, rotationPower);
        }
        else {
            drive(stickHeading, translationPower, rotationPower);
        }
    }

    public double getNecessaryRotationPower(EssentialHeading target, double gain) {
        if (imu == null) {
            throw new IllegalStateException("Cannot calculate necessary rotation power without IMU enabled");
        }
        Orientation orientationNow = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        EssentialHeading headingNow = EssentialHeading.fromInvertedOrientation(orientationNow);
        return getNecessaryRotationPower(target, headingNow, gain);
    }

    public double getNecessaryRotationPower(EssentialHeading target, EssentialHeading current, double gain) {
        if (Math.abs(current.subtract(target).getAngleDegrees()) < 2) {
            return 0;
        }
        EssentialHeading difference = target.subtract(current);
        return Math.min(Math.max(difference.getAngleDegrees() * gain, -0.25),0.25);
    }

    public boolean isFacing(EssentialHeading target) {
        if (imu == null) {
            throw new IllegalStateException("Cannot find direction with IMU disabled");
        }
        Orientation orientationNow = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        EssentialHeading headingNow = EssentialHeading.fromInvertedOrientation(orientationNow);
        return Math.abs(headingNow.subtract(target).getAngleDegrees()) < 2;
    }

    public void driveWithRamper(EssentialHeading heading, double translationPower, double rotationPower) {
        if (!usingRamper) {
            drive(heading, translationPower, rotationPower);
        }
        ramper.ramp(translationPower);
        drive(heading, ramper.getValue(), rotationPower);
    }

    public void drive(EssentialHeading heading, double translationPower, double rotationPower) {
        EssentialHeading headingNow = new EssentialHeading(0);
        if (imu != null) {
            Orientation orientationNow = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            headingNow = EssentialHeading.fromInvertedOrientation(orientationNow);
            Log.d(TAG, "[DRIVEALG] Heading now: " + headingNow.getAngleDegrees());
        }
        EssentialHeading currentRobotHeading = headingNow.subtract(headingStraight);
        Log.d(TAG, "[DRIVEALG] Current robot abs. heading: " + currentRobotHeading.getAngleDegrees());
        EssentialHeading drivingDirection = heading.subtract(currentRobotHeading);
        Log.d(TAG, "[DRIVEALG] Necessary driving direction: " + currentRobotHeading.getAngleDegrees());

        Log.d(TAG, "[DRIVEALG] Got heading: " + heading.getAngleDegrees());
        Log.d(TAG, "[DRIVEALG] Got translation power: " + translationPower);
        Log.d(TAG, "[DRIVEALG] Got rotation power: " + rotationPower);
        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            MotorDescription motorDescription = motorConfiguration.get(i);
            Log.d(TAG, "[DRIVEALG-"+i+"] Now processing for motor with heading" + motorDescription.getEssentialHeading().getAngleDegrees());
            EssentialHeading offset = drivingDirection.subtract(motorDescription.getEssentialHeading()).regularizeToSemicircle();
            Log.d(TAG, "[DRIVEALG-"+i+"] Heading offset: " + offset.getAngleDegrees());
            double headingInducedPowerScale = offset.getAngleDegrees() / 90;
            Log.d(TAG, "[DRIVEALG-"+i+"] Heading-induced power scale: " + headingInducedPowerScale);
            double rotationNecessarySpeed = motorDescription.getRotationGain() * rotationPower;
            Log.d(TAG, "[DRIVEALG-"+i+"] Necessary rotation speed: " + rotationNecessarySpeed);
            Log.d(TAG, "[DRIVEALG-"+i+"] Final (uncapped) power: " + ((translationPower * headingInducedPowerScale) + rotationNecessarySpeed));
            motor.setPower(Math.max(Math.min((translationPower * headingInducedPowerScale) + rotationNecessarySpeed, 1), -1));
            /*telemetry.addData(
                    "* M" + i + ", H" + motorDescription.getEssentialHeading().getAngleDegrees() + ", G" + motorDescription.getRotationGain(),
                    "\t" + motor.getPower()
            );*/
        }
        telemetry.addData("* Fruity Controller", "Driving, H" + heading.getAngleDegrees() + ", T" + translationPower + ", R" + rotationPower);
    }

}

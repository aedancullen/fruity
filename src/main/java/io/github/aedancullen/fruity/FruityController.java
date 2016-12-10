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

    private int[] movedDistanceZero;

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
            headingStraight = EssentialHeading.fromOrientation(orientationStraight);
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

    public void setupRamper(double translationPowerRampRate, double rotationPowerRampRate) {
        ramper = new Ramper(translationPowerRampRate, rotationPowerRampRate);
        usingRamper = true;
    }

    public void disableRamper() {
        usingRamper = false;
    }

    public void handleGamepad(Gamepad gamepad) {
        EssentialHeading headingNow = new EssentialHeading(0);
        if (imu != null) {
            Orientation orientationNow = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            headingNow = EssentialHeading.fromOrientation(orientationNow);
            Log.d(TAG, "[GAMEPAD] Heading now: " + headingNow.getAngleDegrees());
        }
        double stickAngle = Math.toDegrees(Math.atan(gamepad.right_stick_x / -gamepad.right_stick_y));
        if (Double.isNaN(stickAngle)) { stickAngle = 180; }
        if (-gamepad.right_stick_y <= 0) { stickAngle = 180 + stickAngle; }
        EssentialHeading stickHeading = new EssentialHeading(stickAngle);
        Log.d(TAG, "[GAMEPAD] Stick heading: " + stickHeading.getAngleDegrees());
        double translationPower = Math.sqrt(Math.pow(gamepad.right_stick_x,2) + Math.pow(gamepad.right_stick_y,2));
        Log.d(TAG, "[GAMEPAD] Stick deflection (translation power): " + translationPower);
        double rotationPower = gamepad.left_stick_x;
        Log.d(TAG, "[GAMEPAD] Rotation power:" + rotationPower);
        EssentialHeading currentRobotHeading = headingNow.subtract(headingStraight);
        Log.d(TAG, "[GAMEPAD] Current robot abs. heading: " + currentRobotHeading.getAngleDegrees());
        EssentialHeading drivingDirection = stickHeading.subtract(currentRobotHeading);
        Log.d(TAG, "[GAMEPAD] Necessary driving direction: " + currentRobotHeading.getAngleDegrees());
        if (usingRamper) {
            ramper.ramp(translationPower, rotationPower);
            drive(drivingDirection, ramper.getTranslationPower(), ramper.getRotationPower());
        }
        else {
            drive(drivingDirection, translationPower * 0.75, rotationPower * 0.5);
        }
    }

    public double estimateMovedDistance(EssentialHeading heading, double translationPower, double rotationPower) {
        double output = 0;
        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            int movedEnc = motor.getCurrentPosition() - movedDistanceZero[i];
            movedDistanceZero[i] = movedEnc;
            MotorDescription motorDescription = motorConfiguration.get(i);
            EssentialHeading offset = heading.subtract(motorDescription.getEssentialHeading()).regularizeToSemicircle();
            double headingInducedPowerScale = offset.getAngleDegrees() / 90;
            double rotationNecessarySpeed = motorDescription.getRotationGain() * rotationPower;
            double ratio = rotationPower / (translationPower * headingInducedPowerScale);
            double translationPart = movedEnc - (movedEnc * ratio);
            output += translationPart;
        }
        return (output / motors.size());
    }

    public void drive(EssentialHeading heading, double translationPower, double rotationPower) {
        Log.d(TAG, "[DRIVEALG] Got heading: " + heading.getAngleDegrees());
        Log.d(TAG, "[DRIVEALG] Got translation power: " + translationPower);
        Log.d(TAG, "[DRIVEALG] Got rotation power: " + rotationPower);
        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            MotorDescription motorDescription = motorConfiguration.get(i);
            Log.d(TAG, "[DRIVEALG-"+i+"] Now processing for motor with heading" + motorDescription.getEssentialHeading().getAngleDegrees());
            EssentialHeading offset = heading.subtract(motorDescription.getEssentialHeading()).regularizeToSemicircle();
            Log.d(TAG, "[DRIVEALG-"+i+"] Heading offset: " + offset.getAngleDegrees());
            double headingInducedPowerScale = offset.getAngleDegrees() / 90;
            Log.d(TAG, "[DRIVEALG-"+i+"] Heading-induced power scale: " + headingInducedPowerScale);
            double rotationNecessarySpeed = motorDescription.getRotationGain() * rotationPower;
            Log.d(TAG, "[DRIVEALG-"+i+"] Necessary rotation speed: " + rotationNecessarySpeed);
            Log.d(TAG, "[DRIVEALG-"+i+"] Final (uncapped) power: " + ((translationPower * headingInducedPowerScale) + rotationNecessarySpeed));
            motor.setPower(Math.max(Math.min((translationPower * headingInducedPowerScale) + rotationNecessarySpeed, 1), -1));
            telemetry.addData(
                    "* M" + i + ", H" + motorDescription.getEssentialHeading().getAngleDegrees() + ", G" + motorDescription.getRotationGain(),
                    "\t" + motor.getPower()
            );
        }
        telemetry.addData("* Fruity Controller", "Driving, H" + heading.getAngleDegrees() + ", T" + translationPower + ", R" + rotationPower);
    }

}

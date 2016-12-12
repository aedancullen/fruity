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
    EssentialHeading headingNow;
    Ramper ramper;

    double targetRotationAngle = 0;

    double lastStickAngle = NaN;

    private int[] movedDistanceZero;

    public FruityController(HardwareMap hardwareMap,
                            Telemetry telemetry,
                            String imuName,
                            List<DcMotor> motors,
                            DcMotorSimple.Direction clockwiseDirection,
                            DcMotor.RunMode runMode,
                            DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                            List<MotorDescription> motorConfiguration,
                            double translationPowerRampRate,
                            double rotationPowerRampRate
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
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            motor.setDirection(clockwiseDirection);
        }
        movedDistanceZero = new int[motors.size()];
        headingStraight = new EssentialHeading(0);
        String imuStatus;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, imuName);
        imu.initialize(parameters);
        Orientation orientationStraight = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingStraight = EssentialHeading.fromInvertedOrientation(orientationStraight);
        Log.i(TAG, "[CONSTRUCTOR] IMU enabled!");
        imuStatus = imuName;

        Log.i(TAG, "[CONSTRUCTOR] Straight heading: " + headingStraight.getAngleDegrees());
        telemetry.addData("* Fruity Controller", "Initialized " + motors.size() + " motors. IMU: " + imuStatus);

        ramper = new Ramper(translationPowerRampRate, rotationPowerRampRate);
    }

    public void updateHeadingNow() {
        Orientation orientationNow = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingNow = EssentialHeading.fromInvertedOrientation(orientationNow);
        Log.d(TAG, "[GAMEPAD] Heading now: " + headingNow.getAngleDegrees());
    }

    public void handleGamepad(Gamepad gamepad) {
        EssentialHeading headingNow = new EssentialHeading(0);

        double stickAngle = Math.toDegrees(Math.atan(gamepad.right_stick_x / -gamepad.right_stick_y));
        if (-gamepad.right_stick_y <= 0) { stickAngle = 180 + stickAngle; }
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
        double translationPower = Math.sqrt(Math.pow(gamepad.right_stick_x,2) + Math.pow(gamepad.right_stick_y,2));
        Log.d(TAG, "[GAMEPAD] Stick deflection (translation power): " + translationPower);
        //targetRotationAngle += gamepad.left_stick_x;
        //Log.d(TAG, "[GAMEPAD] Rotation power:" + rotationPower);
        EssentialHeading currentRobotHeading = headingNow.subtract(headingStraight);
        Log.d(TAG, "[GAMEPAD] Current robot abs. heading: " + currentRobotHeading.getAngleDegrees());
        EssentialHeading drivingDirection = stickHeading.subtract(currentRobotHeading);
        Log.d(TAG, "[GAMEPAD] Necessary driving direction: " + currentRobotHeading.getAngleDegrees());

        ramper.ramp(translationPower, targetRotationAngle, headingNow.getAngleDegrees());
        driveWithRotationAngle(drivingDirection, ramper.getTranslationPower(), ramper.getRotationPower());
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
            double ratio = rotationNecessarySpeed / (translationPower * headingInducedPowerScale);
            double translationPart = movedEnc - (movedEnc * ratio);
            output += translationPart / headingInducedPowerScale;
        }
        return (output / motors.size());
    }

    public void driveWithRotationAngle(EssentialHeading heading, double translationPower, double rotationAngle) {
        ramper.ramp(translationPower, targetRotationAngle, headingNow.getAngleDegrees());
        driveWithRotationPower(heading, ramper.getTranslationPower(), ramper.getRotationPower());
    }

    public void driveWithRotationPower(EssentialHeading heading, double translationPower, double rotationPower) {
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

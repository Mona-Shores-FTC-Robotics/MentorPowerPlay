package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    public boolean PickUp;
    public boolean Drop;
    public DcMotor intake;
    HardwareMap hwMap = null;
    public boolean stopIntake;
    public boolean reverse;
    public boolean freightCatch;
    public Servo freightStop;
    public DistanceSensor sensorDistance;
    public double yFreight;
    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public boolean lightOn = false;
    public RevBlinkinLedDriver.BlinkinPattern patternOff;

    //private variables
    public final double inPower = 1;
    public final double outPower = .65;
    public final double reverseIntake = -1;
    public final double catchFreight = 0.5;
    /* Constructor */
    public Intake() {

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        intake = ahwMap.get(DcMotor.class, "intake");
        freightStop = hwMap.get(Servo.class, "fStop");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        freightStop.setPosition(0);

        intake.setPower(0);

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "ColorDistanceSensor");

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");

        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        patternOff = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }
//banana
    public void intake() {
        if (PickUp) {

            intake.setPower(inPower);
        }
        if (Drop) {

            intake.setPower(outPower);
            freightStop.setPosition(0);
        }
        if (stopIntake) {
            intake.setPower(0);
        }
        if (reverse) {
            intake.setPower(reverseIntake);
        }
        if (freightCatch) {
            freightStop.setPosition(catchFreight);
        }
    }
    public void RevColorDistanceV3() {
        yFreight = sensorDistance.getDistance(DistanceUnit.CM);
        if (yFreight < 4) {
            lightOn = true;
        }
        else {
            lightOn = false;
        }
    }
    public void Blinkin () {
        if (lightOn) {
            blinkinLedDriver.setPattern(pattern);
        }
        else {
            blinkinLedDriver.setPattern(patternOff);
        }
    }
}
package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmTKL {

    public static final double ARM_INTAKE = 0.5;
    public static final double ARM_LEFT_OUTTAKE = 0;
    public static final double ARM_RIGHT_OUTTAKE = 1.0;
    public DcMotorEx flipArm;
    private int armPosition;
    private double armPower = .5;
    private int forwardOutake = 75;
    private int backwardOuttake = 165;
    private int intake = 20;

    public void init(HardwareMap ahwMap) {
        flipArm = ahwMap.get(DcMotorEx.class, "FlipArm");

        //set arm at intake position
        flipArm.setDirection(DcMotor.Direction.FORWARD);
        flipArm.setPower(0);
        flipArm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        flipArm.setMode(RunMode.RUN_TO_POSITION);
        armPosition = flipArm.getCurrentPosition();

    }

    public void resetArmPosition (){

    }

    public void flip(LinearOpMode activeOpMode, boolean DPadUp, boolean DPadDown, boolean DPadLeft){
        armPosition = flipArm.getCurrentPosition();
        activeOpMode.telemetry.addData("flipArm Encoder Value", armPosition );
        activeOpMode.telemetry.update();

        if(DPadUp){
            flipArm.setTargetPosition(forwardOutake);
            flipArm.setPower(armPower);
        }
        else if (DPadDown){
            flipArm.setTargetPosition(intake);
            flipArm.setPower(armPower);
        }
        else if (DPadLeft){
            flipArm.setTargetPosition(backwardOuttake);
            flipArm.setPower(armPower);
        }
        else {
            flipArm.setPower(0);
        }

    }
}

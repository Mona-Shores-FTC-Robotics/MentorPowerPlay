package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    //public DcMotor Lift;
    public DcMotorEx Lift;
    public int liftPosition;
    HardwareMap hwMap = null;
    public double elevator = 0;
    public boolean elevatorLow = true;
    public boolean elevatorMid;
    public boolean elevatorHigh;
    public int ifRun = 0;

    public final double MaxPower = 0.5;

    public final int low = -20;
    public final int mid = -350;//changed from -450 by MandJ
    public final int high = -850;//changed from -1000 by MandM
    private final int mult = 538;
    public final int increment = 30;
    private boolean runLift = false;



    /* Constructor */
    public Lift(){

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Lift  = ahwMap.get(DcMotorEx.class, "Lift");

        Lift.setDirection(DcMotorSimple.Direction.FORWARD);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setPower(0);
    }

    public void ManualLift() {
        if (elevatorLow || elevatorMid || elevatorHigh) {
            runLift = true;
        }
        if (runLift){
            Lift.setPower(MaxPower);
        }

            liftPosition = Lift.getCurrentPosition();
        //  18-36 lowest position
            if (elevatorLow && liftPosition < -70) {
                Lift.setTargetPosition(low);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ifRun = 1;
            }
        if (elevatorLow && liftPosition > -70) {
            //Lift.setPower(0);
            Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ifRun = 2;
        }
        if (elevatorMid){
            Lift.setTargetPosition(mid);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ifRun = 3;
        }
        if (elevatorHigh){
            Lift.setTargetPosition(high);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ifRun = 4;
        }
        //  -469 middle position?
        //  -1000 top position?


    }

    public void LiftStepDown() {

        Lift.setPower(MaxPower);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPosition = Lift.getCurrentPosition() + increment;
        if (targetPosition > -20){
            targetPosition = -20;
        }
    }

    public void LiftStepUp() {

        Lift.setPower(MaxPower);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPosition = Lift.getCurrentPosition() - increment;
        if (targetPosition < -1000){
            targetPosition = -1000;
        }
    }

}



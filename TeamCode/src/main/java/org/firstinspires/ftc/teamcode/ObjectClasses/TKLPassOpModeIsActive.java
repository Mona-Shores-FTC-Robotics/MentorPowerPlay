package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TKLPassOpModeIsActive {


    /* Constructor */
    public TKLPassOpModeIsActive(){

    }

    public void TestPass(LinearOpMode activeOpMode){
        while (activeOpMode.opModeIsActive()){
            //get stuck in this loop until OpMode end
        }
    }

}

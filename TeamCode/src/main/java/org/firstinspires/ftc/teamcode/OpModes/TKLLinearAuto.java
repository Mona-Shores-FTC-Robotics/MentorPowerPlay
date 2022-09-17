package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.TKLPassOpModeIsActive;

@Autonomous (name = "TKLLinearAuto", group = "LinearOpMode")
// @Disabled

public class TKLLinearAuto extends LinearOpMode
{
    TKLPassOpModeIsActive TestObj = new TKLPassOpModeIsActive();

    @Override
    public void runOpMode() throws InterruptedException {
        TestObj.TestPass(this);
    }

}

package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp_Iterative;

public class ButtonConfig {

    public String startingLocationString;
    public String allianceColorString;
    private int lastStickY;

    /* Constructor */
    public ButtonConfig() {

    }

    public void init() {
        startingLocationString = "Row 2";
        allianceColorString = "Blue";
    }

    public void ConfigureAllianceColor(LinearOpMode activeOpMode) {
        if (activeOpMode.gamepad1.dpad_up) {
            if (allianceColorString.equals("Blue")) {
                allianceColorString = "Red";
            } else if (allianceColorString.equals("Red")) {
                allianceColorString = "Blue";
            }
            activeOpMode.sleep(250);
        }
    }

    public void ConfigureStartingLocation(LinearOpMode activeOpMode) {
        if (activeOpMode.gamepad1.dpad_down) {
            if (startingLocationString.equals("Row 2")) {
                startingLocationString = "Row 5";
            } else if (startingLocationString.equals("Row 5")) {
                startingLocationString = "Row 2";
            }
            activeOpMode.sleep(250);
        }
    }

    public void ConfigureMultiplier(OpMode activeOpMode, DriveTrain MecDrive) {
        if (activeOpMode.gamepad1.left_stick_y > .25 && MecDrive.multiplier > MecDrive.MINMULT && lastStickY != 1) {
            MecDrive.multiplier = (MecDrive.multiplier * 10 - 1) / 10;
            lastStickY = 1;
        }

        else if (activeOpMode.gamepad1.left_stick_y < -.25 && MecDrive.multiplier < MecDrive.MAXMULT && lastStickY != -1) {
            MecDrive.multiplier = (MecDrive.multiplier * 10 + 1) / 10;
            lastStickY = -1;
        }
        else if (activeOpMode.gamepad1.left_stick_y < .25 && activeOpMode.gamepad1.left_stick_y > -.25){
            lastStickY = 0;
        }
        activeOpMode.telemetry.addData("Drive Multiplier", MecDrive.multiplier);
        activeOpMode.telemetry.update();
    }
}

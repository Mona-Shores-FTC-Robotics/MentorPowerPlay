package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.OpenCVWebcam;

@Autonomous(name = "AutoOp")
public class AutoOp extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    OpenCVWebcam Vision = new OpenCVWebcam();
    ButtonConfig ButtonConfig = new ButtonConfig();

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        Vision.init(hardwareMap);
        ButtonConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            ButtonConfig.ConfigureAllianceColor(this);
            ButtonConfig.ConfigureStartingLocation( this);
            telemetry.addData("Alliance Color ", ButtonConfig.allianceColorString);
            telemetry.addData("Starting Location ", ButtonConfig.startingLocationString);
            telemetry.update();
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 1;
        }

        //Autonomous Routine Example
        //MecDrive.encoderDrive(.3, 10, 10, this);
        //MecDrive.strafeDrive(.3, -10, -10, this);
        MecDrive.encoderDrive(.4, 1, 1, this);
        MecDrive.turn(45,this);
        sleep(1000);
        MecDrive.turnTo(0,this);

    }
}




package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.OpenCVWebcam2;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "MentorAuto")
public class MentorAuto extends LinearOpMode {

    DcMotor BL, FL, FR, BR;
    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    final double TICKS_PER_REV = 400.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 3.93701;
    double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    int allianceColor = 1; //1 = Blue // -1 = Red
    int startLocation = 1; //1 = close to audience A2/F2 // -1 =opposite side as audience A5/F5
    int allianceLocationFactor = allianceColor * startLocation;
    int autoNumber = 4; //1 = Just Park; 2 = Deliver One and Park; 3= Deliver Six and Park
    int halfTileDistance = 33;
    int fullTileDistance = 65;
    int Signal;

    OpenCVWebcam2 Vision = new OpenCVWebcam2();

    @Override

    public void runOpMode() {

        DriveTrain MecDrive = new DriveTrain();
        MecDrive.init(hardwareMap);
        Vision.init(hardwareMap);

        FL = hardwareMap.dcMotor.get("LFDrive");
        FR = hardwareMap.dcMotor.get("RFDrive");
        BL = hardwareMap.dcMotor.get("LBDrive");
        BR = hardwareMap.dcMotor.get("RBDrive");

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        String allianceColorString = "Blue";
        String startingLocationString = "Row 2 Near Audience";

        List<Auto> autoList = Arrays.asList(Auto.values());
        Iterator i = autoList.iterator();
        Auto currentAuto = (Auto)i.next();

        while (!isStarted()) {
            telemetry.addData("Alliance Color ", allianceColorString);
            telemetry.addData("Starting Location ", startingLocationString);
            telemetry.addData("Autonomous Routine ", currentAuto);

            //telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData("left square green channel", Vision.LeftMax);
            telemetry.addData("middle square green channel", Vision.MiddleMax);
            telemetry.addData("right square green channel", Vision.RightMax);

            telemetry.addData("Team Element Location", Vision.TeamEleLoc);

            Vision.FinalTeamEleLoc = Vision.TeamEleLoc;

            telemetry.update();
            if (gamepad1.dpad_down) {
                if (allianceColorString.equals("Blue")) {
                    allianceColorString = "Red";
                    allianceColor = -1;
                    sleep(500);
                } else if (allianceColorString.equals("Red")){
                    allianceColorString = "Blue";
                    allianceColor = 1;
                    sleep(500);
                }
            }
            if (gamepad1.dpad_up) {
                if (startingLocationString.equals("Row 2 Near Audience")){
                    startingLocationString = "Row 5 Far from Audience";
                    startLocation = -1;
                    sleep(500);
                } else if (startingLocationString.equals("Row 5 Far from Audience")) {
                    startingLocationString = "Row 2 Near Audience";
                    startLocation = 1;
                    sleep(500);
                }
            }

            if (gamepad1.dpad_left) {
                if (i.hasNext()) {
                    currentAuto = (Auto) i.next();
                    sleep(500);
                }
                else {
                    i = autoList.iterator();
                }
            }
        }

        telemetry.addData("Final Team Element Location", Vision.FinalTeamEleLoc);
        telemetry.update();


        //Read Signal
        //Use Webcam to find out Signal and store in Signal variable
        Signal = 1;

        //Backup into wall for alignment
        //encoderDrive(.8, 20, 20);

        //Just Park Based on Signal

        switch(currentAuto) {
            case JUSTPARK:
                autoJustPark();
                break;
            case SCOREONEANDPARK:
                autoDeliverOneThenPark();
                break;
            case SCORESIXPARK:
                autoDeliverSixThenPark();
                break;
            case JUSTPARKTIME:
                ParkWithTime(MecDrive);
                break;
            case VISIONTURN:
                VisionTurn(MecDrive);
                break;
        }

    }

        public void VisionTurn(DriveTrain MecDrive)
    {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.2;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("middle square green channel", Vision.MiddleMax);
                        telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

        sleep(1000);

        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = -0.2;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;

    }

    public void encoderDrive(double speed, int leftInches, int rightInches) {

        if (opModeIsActive()) {

            int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
            int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftFrontTarget);
            FR.setTargetPosition(newRightFrontTarget);
            BL.setTargetPosition(newLeftBackTarget);
            BR.setTargetPosition(newRightBackTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < 5) &&
                    (FR.isBusy() && FL.isBusy() && BL.isBusy() && BR.isBusy())) {
                telemetry.addData("Encoder BL", FL.getCurrentPosition());
                telemetry.addData("Encoder FR", FR.getCurrentPosition());
                telemetry.addData("Encoder BL", BR.getCurrentPosition());
                telemetry.addData("Encoder BR", BL.getCurrentPosition());

                telemetry.addData("Encoder Target", newLeftFrontTarget);

                telemetry.update();
            }


            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);

        }

    }

    public void strafeDrive(double speed, int leftInches, int rightInches) {

        if (opModeIsActive()) {

            int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
            int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftFrontTarget);
            FR.setTargetPosition(-newRightFrontTarget);
            BL.setTargetPosition(-newLeftBackTarget);
            BR.setTargetPosition(newRightBackTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < 5) &&
                    (FR.isBusy() && FL.isBusy() && BL.isBusy() && BR.isBusy())) {
                telemetry.addData("Encoder BL", FL.getCurrentPosition());
                telemetry.addData("Encoder FR", FR.getCurrentPosition());
                telemetry.addData("Encoder BL", BR.getCurrentPosition());
                telemetry.addData("Encoder BR", BL.getCurrentPosition());
                telemetry.addData("Encoder Target", newLeftFrontTarget);
                telemetry.update();
            }


            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);

        }
    }


    public void autoJustPark() {
        if (opModeIsActive()) {
            encoderDrive(.1, -78, -78);

            if (Signal == 1) {
                strafeDrive(.2, 66, 66);
            } else if (Signal == 2) {
            } else if (Signal == 3) {
                strafeDrive(.2, -66, -66);
            }
        }

    }

    public void autoDeliverOneThenPark() {

        if (opModeIsActive()) {
            //Drive Forward
            encoderDrive(.8, -140, -140);
            //Strafe in Front of High Pole
            //Can we use vision to find the pole?
            strafeDrive(.6, halfTileDistance * allianceLocationFactor, halfTileDistance * allianceLocationFactor);
            //Place Cone on High Pole
            //PLACEHOLDER CODE FOR PLACING CONE
            sleep(1000);

            //Park after placing cone
            if (Signal == 1) {
                strafeDrive(.6, fullTileDistance - (halfTileDistance * allianceLocationFactor), fullTileDistance - (halfTileDistance * allianceLocationFactor));
            } else if (Signal == 2) {
                strafeDrive(.6, -(halfTileDistance * allianceLocationFactor), -(halfTileDistance * allianceLocationFactor));
            } else if (Signal == 3) {
                strafeDrive(.6, ((-1) * (fullTileDistance + halfTileDistance) * allianceLocationFactor), (-1 * (fullTileDistance + halfTileDistance) * allianceLocationFactor));
            }
        }
    }


    public void autoDeliverSixThenPark() {

        if (opModeIsActive()) {
            //Drive Forward
            encoderDrive(.8, -140, -140);
            //Strafe in Front of High Pole
            //Can we use vision to find the pole?
            strafeDrive(.6, halfTileDistance * allianceLocationFactor, halfTileDistance * allianceLocationFactor);
            //Place Cone on High Pole
            //PLACEHOLDER CODE FOR PLACING CONE
            sleep(2000);

            //Drive to Cones by the Wall
            strafeDrive(.6, (-1 * halfTileDistance * allianceLocationFactor), (-1 * halfTileDistance * allianceLocationFactor));

            // Turn to align with cones [use color sensor later?]
            encoderDrive(.5, -65 * allianceLocationFactor, 65 * allianceLocationFactor);
            //drive to Stacked Cone line
            encoderDrive(.5, (-1 * allianceLocationFactor * halfTileDistance), (-1 * allianceLocationFactor * halfTileDistance));
            //routine to pickup cone, deliver cone, and return to red line
            PickupConeFromStack();


        }
    }

    public void PickupConeFromStack() {
        //approach cone
        encoderDrive(.5, (-1 * allianceLocationFactor * halfTileDistance), (-1 * allianceLocationFactor * halfTileDistance));
        //drive right up to cone and align on wall?
        encoderDrive(.5, (-1 * allianceLocationFactor * halfTileDistance), (-1 * allianceLocationFactor * halfTileDistance));
        //PLACEHOLDER CODE FOR PICKING UP CONE
        sleep(2000);

        //backup
        encoderDrive(.5, (allianceLocationFactor * halfTileDistance), (allianceLocationFactor * halfTileDistance));

        encoderDrive(.5, (allianceLocationFactor * (fullTileDistance + halfTileDistance)), (allianceLocationFactor * (fullTileDistance + halfTileDistance)));
        // Turn to align with cones [use color sensor later?]
        encoderDrive(.5, 65 * allianceLocationFactor, -65 * allianceLocationFactor);
        //PLACEHOLDER CODE FOR PLACING CONE
        sleep(2000);
        // Turn to align with cones [use color sensor later?]
        encoderDrive(.5, -65 * allianceLocationFactor, 65 * allianceLocationFactor);
        //drive to Stacked Cone line
        encoderDrive(.5, (-1 * allianceLocationFactor * halfTileDistance), (-1 * allianceLocationFactor * halfTileDistance));
    }

    public void ParkWithTime(DriveTrain MecDrive) {

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MecDrive.drive = 0.2;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

    }

        public enum Auto {
            VISIONTURN, JUSTPARK, SCOREONEANDPARK, SCORESIXPARK, JUSTPARKTIME;
        }



    }


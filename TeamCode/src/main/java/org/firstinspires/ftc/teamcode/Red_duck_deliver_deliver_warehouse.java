/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red_Duck_Deliver_Deliver_Warehouse", group="Pushbot")
//@Disabled
//Arm is the only thing that needs to be adjusted
public class Red_duck_deliver_deliver_warehouse extends LinearOpMode {

    /* Declare OpMode members. */
    // HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    DriveTrain MecDrive = new DriveTrain();
    private ElapsedTime runtime = new ElapsedTime();
    Intake intake = new Intake();
    CarouselDuck spinner = new CarouselDuck();
    Lift lift = new Lift();
    private double firstTurn = -40;
    private double secondTurn = 22; //changed from 20 to 22 based on 12/17 video
    private double ThirdTurn = -85;
    private long sleeptime = 1000;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private double outPower = -0.45;
    public long holdOn = 1000;

    OpenCVWebcam2 Vision = new OpenCVWebcam2();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        MecDrive.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        spinner.init(hardwareMap);

       Vision.init(hardwareMap);
        spinner.DuckArm.setPosition(spinner.arm);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData("left square green channel", Vision.LeftMax);
            telemetry.addData("middle square green channel", Vision.MiddleMax);
            telemetry.addData("right square green channel", Vision.RightMax);
            //telemetry.addData("Adjusted Threshold", Vision.AdjustedThreshold);
            //telemetry.addData("Unadjusted Threshold", Vision.UnadjustedThreshold);
            telemetry.addData("Team Element Location", Vision.TeamEleLoc);
            telemetry.update();
            Vision.FinalTeamEleLoc = Vision.TeamEleLoc;
        }
        Vision.webcam.stopStreaming();
        telemetry.addData("Final Team Element Location", Vision.FinalTeamEleLoc);
        telemetry.update();


        spinner.DuckArm.setPosition(spinner.rest);
        spinner.DuckSpinner.setPower(-0.7);
        spinner.carouselDuck();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.5)) {

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }
        //spinner.stopSpinner = true;
        spinner.DuckSpinner.setPower(0);
        spinner.DuckArm.setPosition(spinner.rest);

        sleep(2000);

 //turn to hub
       turn(firstTurn);

//go forward
        MecDrive.drive = 0.4;
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

//Hub turn
        turn(secondTurn);

//lift by vision
        lift.ManualLift();
        if (Vision.FinalTeamEleLoc == 0) {
            lift.Lift.setTargetPosition(lift.low);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Vision.FinalTeamEleLoc == 1) {
            lift.Lift.setTargetPosition(lift.mid);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Vision.FinalTeamEleLoc == 2) {
            lift.Lift.setTargetPosition(lift.high);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        sleep(sleeptime);
//go forward
        MecDrive.drive = 0.4;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .31)) { //changed from .25 to .31
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

//drop freight
        intake.intake();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) { // changed 1.5 to 1.2
            //intake.Drop = true;
            intake.intake.setPower(.65);
        }
        //intake.stopIntake = true;
        intake.intake.setPower(0);

        sleep(holdOn);

//Outtake slowly
        intake.intake();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            intake.intake.setPower(outPower);
        }

        intake.intake.setPower(0);

//going backwards
        MecDrive.drive = -0.4;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//drop lift
        lift.Lift.setTargetPosition(lift.low);
        lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
//turn left
       turn(ThirdTurn);

//strafe to wall

        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.35;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)){
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();


//Set the freight catch

intake.freightCatch = true;

        //intake freight
        intake.intake();
        MecDrive.drive = 0.34;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.7)) {
            //intake.Drop = true;
            intake.intake.setPower(1);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }

        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

        //intake.stopIntake = true;
        intake.intake.setPower(0);

        //go backward
        MecDrive.drive = -0.4;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .98)) { //changed from .95 to .98
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

        //turn
        turn(22);  //this should be FourthTurn, changed from 20 to 22

        //lift by vision

        lift.ManualLift();
        //if (Vision.FinalTeamEleLoc == 0) {
        //    lift.Lift.setTargetPosition(lift.low);
        //    lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}
        //if (Vision.FinalTeamEleLoc == 1) {
        //    lift.Lift.setTargetPosition(lift.mid);
        //    lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}
        //if (Vision.FinalTeamEleLoc == 2) {
            lift.Lift.setTargetPosition(lift.high);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}

        //drive forward
        MecDrive.drive = 0.4;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.05)) { //changed from 1.15 to 1.05
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

        //Drop freight
        intake.intake();
        intake.freightStop.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) { //changed from 1.5 to 1.2
            //intake.Drop = true;
            intake.intake.setPower(.65);
        }

        sleep(holdOn);

        //Outtake slowly
        intake.intake();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            intake.intake.setPower(outPower);
        }

        intake.intake.setPower(0);

        //intake.stopIntake = true;
        intake.intake.setPower(0);

        //turn
        turn(-67.5);  //changed from -67 to -67.5

        //Lower lift
        lift.Lift.setTargetPosition(lift.low);
        lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //drive forward
        MecDrive.drive = 0.8;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.9)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        MecDrive.drive = 0.0;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();

    }
    //  Start Gyro methods copy
    double gyroTurnMin = 0.25;
    double gyroTurnMax = 1;
    public double getAngle(){
        Orientation orientation = MecDrive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if(deltaAngle > 180){
            deltaAngle -= 360;
        } else if(deltaAngle <= -180){
            deltaAngle += 360;
        }
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("Gyro", orientation.firstAngle);
        telemetry.addData("where am I", currAngle);
        return currAngle;
    }

    public void turn(double degrees){
        MecDrive.resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > 2){
            double setPower = Range.clip(Math.abs(error)/180+gyroTurnMin, -gyroTurnMax, gyroTurnMax);
            double motorPower = (error < 0 ? -setPower: setPower);
            MecDrive.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        MecDrive.setAllPower(0);
        MecDrive.MecanumDrive();
    }

    // end gyro method copy
}


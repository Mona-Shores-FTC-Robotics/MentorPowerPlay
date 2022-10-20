package org.firstinspires.ftc.teamcode.OpModes;


/*import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_RIGHT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_INTAKE_HEIGHT_CHANGE_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.W_3_JUNCTION;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.X_2_JUNCTION;
*/
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ArmTKL;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
//import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
//import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
//import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;

@TeleOp(name = "Teleop Mode w/ Turret Bot", group = "Turret Bot")
public class TeleOp_Linear_FlipArm_Bot extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();
    ArmTKL FlipArm = new ArmTKL();
    //  Intake ServoIntake = new Intake();
    //  Claw ServoClaw = new Claw();
    //   Lift Lift = new Lift();

    boolean operatorA = false;
    boolean operatorB = false;
    boolean operatorX = false;

    private final ElapsedTime runtime = new ElapsedTime();

    private int teleopConeDeliveryTracker = 0;

    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        FlipArm.init(hardwareMap);
        //  ServoIntake.init(hardwareMap);
        //  ServoClaw.init(hardwareMap);
        //Lift.init(hardwareMap);
        // Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM,this);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press Start When Ready", "");

        while (!isStarted()) {
            ButtonConfig.ConfigureMultiplier(this, MecDrive);
        }

        runtime.reset();

        while (opModeIsActive()) {
            //Read gamepad and robot state
            MecDrive.drive = -gamepad1.left_stick_y; //-1.0 to 1.0
            MecDrive.strafe = gamepad1.left_stick_x; //-1.0 to 1.0
            MecDrive.turn = gamepad1.right_stick_x; //-1.0 to 1.0
            operatorX = gamepad1.x;
            operatorA = gamepad1.a;
            operatorB = gamepad1.b;

            //Method for driving with mecanum chassis
            MecDrive.MecanumDrive();
            //Method for moving the arm between intake, front outtake and back outtake positions
            FlipArm.flip(this, operatorX, operatorA, operatorB);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Motors", "leftfront(%.2f), rightfront (%.2f)", MecDrive.leftFrontPower, MecDrive.rightFrontPower);
            telemetry.addData("Motors", "leftback (%.2f), rightback (%.2f)", MecDrive.leftBackPower, MecDrive.rightBackPower);
            telemetry.addData("# of Cones Delivered", teleopConeDeliveryTracker);
            telemetry.update();
        }

        // Sets all drive motors to 0 when exiting the opmode
        MecDrive.drive = 0;
        MecDrive.strafe = 0;
        MecDrive.turn = 0;

        MecDrive.MecanumDrive();
    }
}




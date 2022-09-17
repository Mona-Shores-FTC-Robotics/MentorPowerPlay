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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;

public class CarouselDuck {

    public Servo DuckArm;
    public DcMotor DuckSpinner = null;
    public boolean armOut;
    public boolean armIn;
    public boolean duckSpinner;
    public boolean duckSpinnerRev;
    public boolean stopSpinner;
    public double spinnerPower = 1;
    public double rest = 0.1;
    public double arm = 0.9;
    public double armPos = 0;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public CarouselDuck() {


    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize ALL installed servos.
        DuckArm = hwMap.get(Servo.class, "DuckArm");
        DuckSpinner = hwMap.get(DcMotor.class, "Spinner");

        DuckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        DuckSpinner.setPower(0);


    }

    public void carouselDuck() {
       /* if (armPos > 0.1) {
            DuckArm.setPosition(arm);
        }
        if (armPos < 0.1) {
            DuckArm.setPosition(rest);
        } */
        if (duckSpinner) {
            DuckSpinner.setPower(spinnerPower);
        }
        if (duckSpinnerRev) {
            DuckSpinner.setPower(-spinnerPower);
        }
        if (stopSpinner) {
            DuckSpinner.setPower(0);
        }

    }

    public void DuckArmSet() {
        if (Math.abs(armPos) > 0.1){
            DuckArm.setPosition(Math.ceil(armPos * 0.5));
        }
    }
}

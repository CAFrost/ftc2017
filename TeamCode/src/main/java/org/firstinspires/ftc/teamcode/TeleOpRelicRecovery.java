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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="T:Relic Recovery", group="2017")
public class TeleOpRelicRecovery extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeftFront = null;
    private DcMotor motorLeftRear= null;
    private DcMotor motorRightFront = null;
    private DcMotor motorRightRear = null;
    private DcMotor motorArm = null;

    private Servo servoClawLeft = null;
    double servoPosition = 0;
    private Servo servoClawRight = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorLeftFront  = hardwareMap.get(DcMotor.class, "m1");
        motorLeftRear  = hardwareMap.get(DcMotor.class, "m2");
        motorRightFront = hardwareMap.get(DcMotor.class, "m3");
        motorRightRear = hardwareMap.get(DcMotor.class, "m4");
        motorArm = hardwareMap.get(DcMotor.class, "ma");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightRear.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        servoClawLeft = hardwareMap.get(Servo.class, "s1");
        servoClawRight = hardwareMap.get(Servo.class, "s2");

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void loop() {
        double leftPower;
        double rightPower;
        double armPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -0.9, 0.9) ;
        rightPower   = Range.clip(drive - turn, -0.9, 0.9) ;

        // Send calculated power to wheels
        motorLeftFront.setPower(leftPower);
        motorLeftRear.setPower(leftPower);
        motorRightFront.setPower(rightPower);
        motorRightRear.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        armPower = -gamepad2.left_stick_y;
        armPower = Range.clip(armPower, -0.05, 0.6);
        if (armPower < 0.25 && armPower > -0.01) {
            armPower=0.20;
        }
        motorArm.setPower(armPower);
        telemetry.addData("Arm", " (%.2f)", armPower);


        boolean clawIsOpen=false;
        //boolean
        boolean buttonOpen=gamepad2.a;
        boolean buttonClose=gamepad2.b;
        telemetry.addData("Buttons", "Open (%b), Close (%b)", buttonOpen, buttonClose);

        if (buttonClose)
        {
            closeClaw();
        }
        else if (buttonOpen)
        {
            openClaw();
        }
    }

    public void closeClaw(){
        servoClawLeft.setPosition(0);
        servoClawRight.setPosition(1);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 0, 1);
    }
    public void openClaw(){
        servoClawLeft.setPosition(1);
        servoClawRight.setPosition(0);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 1, 0);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        motorLeftFront.setPower(0);
        motorLeftRear.setPower(0);
        motorRightFront.setPower(0);
        motorRightRear.setPower(0);
        motorArm.setPower(0);
    }

}

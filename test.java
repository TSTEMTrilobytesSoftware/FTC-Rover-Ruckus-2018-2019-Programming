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

//import necessary classes for program to run

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * @author 12820 Wagner High School FTC Robotics Programming Team: Ian Fernandes, Chloe Price, Aden Briano, Bella Gonzalez, Ruby Gomez, Daniel Coronado
 * @version 1.0
 * This program isn't to be used for competition. However, it is designed so that code from other
 * autonomous programs can be copied into here. This program is special because it allows the
 * user to go step by step in each instruction by pressing b on gamepad1. This allows the programmer
 * to examine the move, make any changes to the original autonomous if necessary, and then continue
 * to the next command.
 */

@TeleOp(name = "Test Program", group = "Linear Opmode")
@Disabled
public class test extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor linearSlide = null;
    private CRServo armServo = null;
    private Servo frontServo = null;

    private double leftPower = 0;
    private double rightPower = 0;
    private double linearPower = 0;
    private double armServoPosition = 0;
    private double frontServoPosition = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        armServo = hardwareMap.get(CRServo.class, "arm_servo");
        frontServo = hardwareMap.get(Servo.class, "front_servo");

        // It is typical for most robots to need one of the motors reversed in order to allow for proper operation.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);

        //set the zero power behavior of the linear slide to BRAKE so that it doesn't coast while hanging on the lander.
        linearSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //list of commands, detailed in class javadoc
        moveLS(2500, -.3);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        goStraight(80, -1);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        goLeft(420);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        goStraight(1900, 1);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        goLeft(1200);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        setFrontServo(0.3);

        while (!gamepad1.b) {
        }

        sleep(2000);

        while (!gamepad1.b) {
        }

        //goLeft(450);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        goStraight(4000, 1);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        moveLS(200, .7);

        while (!gamepad1.b) {
        }

        sleep(1000);

        while (!gamepad1.b) {
        }

        goStraight(1000, 0);//see if rh;obot brings down linear slide
        stopRobot();
    }

    public void updateT() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left: (%.2f), right: (%.2f), linear slide: (%.2f)", leftPower, rightPower, linearPower);
        telemetry.addData("Arm Motors", "linear slide: (%.2f)", linearPower);
        telemetry.addData("Servos", "arm servo: (%.2f), front servo: (%.2f)", armServoPosition, frontServoPosition);
        telemetry.update();
    }

    public void goStraight(int time, double power) {
        leftDrive.setPower(power);
        leftPower = leftDrive.getPower();
        rightDrive.setPower(power);
        rightPower = rightDrive.getPower();
        updateT();
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void goRight(int time) {
        leftDrive.setPower(1);
        leftPower = leftDrive.getPower();
        rightDrive.setPower(-1);
        rightPower = rightDrive.getPower();
        updateT();
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void goLeft(int time) {
        leftDrive.setPower(-1);
        leftPower = leftDrive.getPower();
        rightDrive.setPower(1);
        rightPower = rightDrive.getPower();
        updateT();
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void stopRobot() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        linearSlide.setPower(0);
        armServo.setPower(0);
        leftPower = leftDrive.getPower();
        rightPower = rightDrive.getPower();
        linearPower = linearSlide.getPower();
        armServoPosition = armServo.getPower();
        updateT();
    }

    public void moveLS(int time, double power) {
        if (power >= -1 && power <= 1)
            linearSlide.setPower(power);
        else
            linearSlide.setPower(.5);
        linearPower = linearSlide.getPower();
        updateT();
        sleep(time);
        linearSlide.setPower(0);
    }

    public void setArmServo(double position) {
        if (position >= 0 && position <= 1)
            armServo.setPower(position);
        else
            armServo.setPower(0);
        armServoPosition = armServo.getPower();
        updateT();
    }

    public void setFrontServo(double position) {
        if (position >= 0 && position <= 1)
            frontServo.setPosition(position);
        else
            frontServo.setPosition(0);
        frontServoPosition = frontServo.getPosition();
        updateT();
    }

}

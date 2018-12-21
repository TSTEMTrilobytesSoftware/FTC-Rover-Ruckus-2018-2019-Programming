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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * @author 12820 Wagner High School FTC Robotics Programming Team: Ian Fernandes, Chloe Price, Aden Briano, Bella Gonzalez, Ruby Gomez, Daniel Coronado
 * @version 1.0
 * This program corresponds to the Autonomous portion of the FTC match.
 * This program includes algorithms for 2 drive motors, 1 arm motor, 2 servos, and a color sensor.
 * Description of Autonomous Operations:
 * Robot unhooks from lander.
 * Robot drops off team marker and drives to crater.
 */

@Autonomous(name = "No Color Sense Go Through Minerals LEFT", group = "Linear Opmode")
public class Working12820AutoDriveThroughMinerals extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor linearSlide = null;
    private DcMotor armServo = null;
    private Servo frontServo = null;
    private DcMotor downMotor = null;

    private double leftPower = 0;
    private double rightPower = 0;
    private double linearPower = 0;
    private double armServoPower = 0;
    private double frontServoPosition = 0;
    private double downMotorPower = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        armServo = hardwareMap.get(DcMotor.class, "arm_servo");
        frontServo = hardwareMap.get(Servo.class, "front_servo");
        downMotor = hardwareMap.get(DcMotor.class, "down_motor");


        // It is typical for most robots to need one of the motors reversed in order to allow for proper operation.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(DcMotor.Direction.FORWARD);
        downMotor.setDirection(DcMotor.Direction.REVERSE);


        //set the zero power behavior of the linear slide to BRAKE so that it doesn't coast while hanging on the lander.
        linearSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        downMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //list of commands, detailed in class javadoc
        moveLS(2500, -.3);

        sleep(1000);

        goStraight(80, -1);

        sleep(1000);

        goLeft(500);

        sleep(1000);

        goStraight(2100, 1);

        sleep(1000);

        goLeft(750);

        sleep(1000);

        setFrontServo(0.3);

        sleep(3000);

        goStraight(4000, 1);

        sleep(1000);

        moveLS(200, .7);

        sleep(1000);

        goStraight(1000, 0);//see if rh;obot brings down linear slide

        setArmMotor(1);

        sleep(1000);

        setArmMotor(0);

        stopRobot();
    }

    /**
     * Updates the telemetry of the driver station with data from motors and servos
     */
    public void updateT() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left: (%.2f), right: (%.2f), linear slide: (%.2f)", leftPower, rightPower, linearPower);
        telemetry.addData("Arm Motors", "linear slide: (%.2f)", linearPower);
        telemetry.addData("Servos", "arm motor: (%.2f), front servo: (%.2f)", armServoPower, frontServoPosition);
        telemetry.update();
    }

    /**
     * Sets the power of both drive motors to that specified for the duration of time specified.
     * @param time  in milliseconds
     * @param power from -1 to 1
     */
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

    /**
     * Alternates the power of the drive motors so that the robot makes a right turn in its place for a
     * duration of time specified by the user.
     *
     * @param time in milliseconds
     */
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

    /**
     * Alternates the power of the drive motors so that the robot makes a left turn in its place for a
     * duration of time specified by the user.
     *
     * @param time in milliseconds
     */
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

    /**
     * Resets the motors and servos on the robot so that it is in "ready" position.
     */
    public void stopRobot() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        linearSlide.setPower(0);
        armServo.setPower(0);
        leftPower = leftDrive.getPower();
        rightPower = rightDrive.getPower();
        linearPower = linearSlide.getPower();
        armServoPower = armServo.getPower();
        updateT();
    }

    /**
     * Sets the power of the linear slide to that specified for the duration of time, which is also
     * specified.
     *
     * @param time  in milliseconds
     * @param power from -1 to 1
     */
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
    /**
     * Sets the power of the down linear slide to that specified for the duration of time, which is also
     * specified.
     *
     * @param time  in milliseconds
     * @param power from -1 to 1
     */
    public void moveDownLS(int time, double power) {
        if (power >= -1 && power <= 1)
            downMotor.setPower(power);
        else
            downMotor.setPower(.5);
        downMotorPower = downMotor.getPower();
        updateT();
        sleep(time);
        downMotor.setPower(0);
    }
    /**
     * Sets the position of the arm servo to that speicifed.
     *
     * @param power from 0 to 1
     */
    public void setArmMotor(double power) {
        if (power >= -1 && power <= 1)
            armServo.setPower(power);
        else
            armServo.setPower(0);
        armServoPower = armServo.getPower();
        updateT();
    }

    /**
     * Sets the position of the front servo to that specified.
     *
     * @param position from 0 to 1
     */
    public void setFrontServo(double position) {
        if (position >= 0 && position <= 1)
            frontServo.setPosition(position);
        else
            frontServo.setPosition(0);
        frontServoPosition = frontServo.getPosition();
        updateT();
    }

}

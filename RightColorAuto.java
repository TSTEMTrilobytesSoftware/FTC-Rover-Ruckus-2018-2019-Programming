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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
 * Robot samples the minerals, moving the gold mineral out of position.
 * Robot drops off team marker and drives to crater.
 */

@Autonomous(name = "Color Sense RIGHT", group = "Linear Opmode")
public class RightColorAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor linearSlide = null;
    private CRServo armServo = null;
    private Servo frontServo = null;
    private ColorSensor cSensor = null;

    // Declare variables for driver station telemetry.
    private double leftPower = 0;
    private double rightPower = 0;
    private double linearPower = 0;
    private double armServoPosition = 0;
    private double frontServoPosition = 0;

    // When initialized...
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
        cSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // It is typical for most robots to need one of the motors reversed in order to allow for proper operation.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);

        //set the zero power behavior of the linear slide to BRAKE to avoid coasting when hanging off the lander.
        linearSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //commands to be executed, detailed by class javadoc
        moveLS(500,-.3);

        sleep(1000);

        goStraight(200,-1);

        sleep(1000);

        goLeft(500);//turn left extra to get to left most mineral

        sleep(1000);

        goStraight(1000, 1); //go to first mineral

        sleep(1000);

        goRight(470);  //face color sensor to first mineral

        sleep(1000);

        //start gold mineral search
        ElapsedTime hey = new ElapsedTime();
        hey.reset();
        hey.startTime();
        while(!(cSensor.red()>=130&&cSensor.red()<=150&&cSensor.green()>=100&&cSensor.green()<=120&&cSensor.blue()>=50&&cSensor.blue()<=65)&&hey.seconds()<3){
            leftDrive.setPower(.2);
            rightDrive.setPower(.2);
            updateT();
        }
        hey.reset();
        updateT();

        //turn in and then back to hit the element out of place
        leftDrive.setPower(-1);
        rightDrive.setPower(1);
        sleep(200);
        leftDrive.setPower(1);
        rightDrive.setPower(-1);
        sleep(200);
        leftDrive.setPower(0);
        rightDrive.setPower(0);


        sleep(1000);

        goStraight(600,1);

        sleep(1000);

        goLeft(470);

        sleep(1000);

        goStraight(1000,1);

        sleep(1000);

        goLeft(200);

        sleep(1000);

        goStraight(3500,1);

        sleep(1000);

        goLeft(940);

        sleep(1000);

        setFrontServo(0.3);

        sleep(2000);

        sleep(1000);

        goStraight(5000,1);

        sleep(1000);

        moveLS(200,.7);

        sleep(2000);

        goStraight(0,1);//see if robot brings down linear slide

        stopRobot();
    }

    /**
     * updateT takes no parameters and updates the telemetry of the driver station based on
     * motor, servo, and sensor data.
     */
    public void updateT() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left: (%.2f), right: (%.2f), linear slide: (%.2f)", leftPower, rightPower, linearPower);
        telemetry.addData("Arm Motors", "linear slide: (%.2f)", linearPower);
        telemetry.addData("Servos", "arm servo: (%.2f), front servo: (%.2f)", armServoPosition, frontServoPosition);
        telemetry.addData("Sensors", "color sensor: red:(%d) green:(%d) blue:(%d) alpha:(%d) argb:(%d) ", cSensor.red(),cSensor.green(),cSensor.blue(),cSensor.alpha(),cSensor.argb());
        telemetry.update();
    }

    /**
     * goStraight takes on two parameters, one for the duration of the operation in milliseconds, and
     * the other for the power to set the drive motors to.
     * This method makes both drive motors turn on at the speed specified for the time specified.
     * @param time in milliseconds
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
     * goRight takes on one parameter, that being time, which indicates the duration of the turn in
     * milliseconds.
     * This method makes the robot turn in place by alternating the values of the left and right
     * drive motors for the time specified.
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
     * goLeft takes on one parameter, that being time, which indicates the duration of the turn in
     * milliseconds.
     * This method makes the robot turn in place by alternating the values of the left and right
     * drive motors for the time specified.
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
     * stopRobot takes on no parameters and completely resets the robot by resetting
     * each hardware component of the robot.
     */
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

    /**
     * moveLS takes on two parameters, one being time(in milliseconds), and the other being power.
     * This method sets the power of the linear slide motor to the power specified for the duration
     * specified.
     * @param time in milliseconds
     * @param power from -1 to 1
     */
    public void moveLS (int time, double power){
        if(power>=-1&&power<=1)
            linearSlide.setPower(power);
        else
            linearSlide.setPower(.5);
        linearPower = linearSlide.getPower();
        updateT();
        sleep(time);
        linearSlide.setPower(0);
    }

    /**
     * setArmServo takes on one parameter, that being position.
     * This method sets the power of the continuous arm servo to the position specified.
     * @param position from 0 to 1
     */
    public void setArmServo(double position) {
        if(position>=0&&position<=1)
            armServo.setPower(position);
        else
            armServo.setPower(0);
        armServoPosition = armServo.getPower();
        updateT();
    }

    /**
     * setFrontServo takes on one parameter, that being position.
     * This method sets the power of the front servo to the position specified.
     * @param position from 0 to 1
     */
    public void setFrontServo(double position) {
        if(position>=0&&position<=1)
            frontServo.setPosition(position);
        else
            frontServo.setPosition(0);
        frontServoPosition = frontServo.getPosition();
        updateT();
    }

}

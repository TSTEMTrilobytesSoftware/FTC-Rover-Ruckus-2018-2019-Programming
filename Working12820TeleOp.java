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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * @author 12820 Wagner High School FTC Robotics Programming Team: Ian Fernandes, Chloe Price, Aden Briano, Bella Gonzalez, Ruby Gomez, Daniel Coronado
 * @version 1.0
 * This program corresponds to the TeleOp portion of the FTC match.
 * This program includes algorithms for 2 drive motors, 1 arm motor, 1 servo, 1 distance sensor, and 1 color sensor.
 * Description of TeleOp Controls:
 * Use Joystick Controller 1(Recommended: Logitech F310) joysticks to engage in tank drive.
 * The left and right joysticks are used to maneuver the robot about the gamefield. (left controls left motion while right controls right motion)
 * Use Joystick Controller 2(Recommended: Logitech F310) joysticks to control any "arm" motion.
 * To make the linear slide go up, press down on the right trigger of gamepad2. To make the linear slide go down, press down on the left trigger of gamepad2.
 * To make the arm servo go forward, move the left joystick up on gamepad2. To make the arm servo go backward, move the left joystick down on gamepad2.
 * To operate the front servo, press the y button on gamepad2 to move it down. As soon as it is released, it will go back to its default position.
 * Press the b button on gamepad2 to activate the braking mechanism on the linear slide in case it is coasting while hooked on to the lander.
 */

@TeleOp(name="Basic TeleOp Linear OpMode for 2018-2019 Season", group="Linear Opmode")
public class Working12820TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor linearSlide = null;
    private DcMotor downMotor = null;
    //private CRServo armServo = null;
    private DcMotor armServo = null;
    private Servo frontServo = null;
    private ColorSensor cSensor = null;
    private DistanceSensor dSensor = null;

    // Sets up a variable for each motor, servo, and sensor to save power levels and other data for telemetry
    double leftPower = 0;
    double rightPower = 0;
    double linearPower = 0;
    double downPower = 0;
    double armServoPower = 0;
    double frontServoPosition = 0;
    boolean brakeOn = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        // "deviceName"s must correspond to the names assigned in the robot configuration file on the robot controller phone.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");              //set up left motor location on robot
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");             //set up right motor location on robot
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");      //set up linear slide motor location on robot
        downMotor = hardwareMap.get(DcMotor.class, "down_motor");

        frontServo = hardwareMap.get(Servo.class, "front_servo");               //set up front servo location on robot
        cSensor = hardwareMap.get(ColorSensor.class, "color_sensor");           //set up color sensor location on robot
        dSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");     //set up distance sensor location on robot
        armServo = hardwareMap.get(DcMotor.class,"arm_servo" );                 //set up arm motor location on robot

        // It is typical for most robots to need one of the motors reversed in order to allow for proper operation.
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        downMotor.setDirection(DcMotor.Direction.REVERSE);
        armServo.setDirection(DcMotor.Direction.FORWARD);

        cSensor.enableLed(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateVars();
            updateSystem();
            updateT();
            //idle();

        }
    }

    public void updateVars(){
        // Tank Drive
        leftPower  = gamepad1.left_stick_y ;
        rightPower = gamepad1.right_stick_y ;

        //Linear Slide
        if(gamepad2.left_trigger==1&&gamepad2.right_trigger==0) {
            //downPower = -.5;
            linearPower = 1;
        }
        else if(gamepad2.left_trigger==0&&gamepad2.right_trigger==1){
            linearPower = -1;
            //downPower = -.5;
        }
        else{
            linearPower = 0;
            downPower = 0;
        }

        //Servos
        armServoPower =  -gamepad2.left_stick_y ;

        if(gamepad2.y)
            frontServoPosition = 0;
        else
            frontServoPosition = 1;





    }

    public void setBrake() {
        if(!brakeOn) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            brakeOn = true;
        }else{
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            brakeOn = false;
            }
        }
        //Ignore...these are RGB values for testing the color sensor
        //13 7 4
        //12 7 4
        //14 8 4
    //15 9 5
    //742  distance sensor value from farther
    //75-90 65-80, 30-40 //average RGB values for yellow

    public void updateSystem() {
        // Send calculated power to respective robot systems
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        linearSlide.setPower(linearPower);
        downMotor.setPower(downPower);
       /* if(-gamepad2.left_stick_y==1) {
            armServo.setDirection(CRServo.Direction.REVERSE);
            armServo.setPower(1);
        }
        else if(-gamepad2.left_stick_y==-1) {


            armServo.setDirection(CRServo.Direction.FORWARD);
            armServo.setPower(1);
        }
        else
            armServo.setPower(0);*/
        frontServo.setPosition(frontServoPosition);
        if(gamepad2.b)
            setBrake();

        armServo.setPower (armServoPower);
    }
    public void updateT() {
        // Show the elapsed game time, wheel power, linear slide power, shovel motor power, servo position, and sensory data.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left: (%.2f), right: (%.2f)", leftPower, rightPower);
        telemetry.addData("Arm Motors", "linear slide: (%.2f)", linearPower );
        telemetry.addData("Servos", "arm servo: (%.2f), front servo: (%.2f)", armServoPower, frontServoPosition);
        telemetry.addData("Sensors", "distance sensor: (%.2f)" , dSensor.getDistance(DistanceUnit.METER));
        telemetry.addData("Sensors", "color sensor: red:(%d) green:(%d) blue:(%d) alpha:(%d) argb:(%d) ", cSensor.red(),cSensor.green(),cSensor.blue(),cSensor.alpha(),cSensor.argb());
        telemetry.update();
    }

    public void updateT(String input)  {
        telemetry.addLine(input);
    }
}

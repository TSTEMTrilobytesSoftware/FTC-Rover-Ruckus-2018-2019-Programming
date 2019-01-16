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
 * @author 12820 Wagner High School FTC Robotics Programming Team: Ian Fernandes, Aden Briano, Bella Gonzalez, Daniel Coronado
 * @version 3.0
 * This program corresponds to the TeleOp portion of the FTC match.
 * This program includes algorithms for 2 drive motors, 1 arm motor, 1 servo, 1 distance sensor, and 1 color sensor.
 * Description of TeleOp Controls:
 * Use Joystick Controller 1(Recommended: Logitech F310) joysticks to engage in tank drive.
 * The left and right joysticks are used to maneuver the robot about the gamefield. (left controls left motion while right controls right motion)
 * Use Joystick Controller 2(Recommended: Logitech F310) joysticks to control any "arm" motion.
 * To make the linear slide go up, press down on the right trigger of gamepad2. To make the linear slide go down, press down on the left trigger of gamepad2.
 * To make the intake go forward, move the left joystick up on gamepad2. To make the intake go backward, move the left joystick down on gamepad2.
 * To operate the front servo, press the y button on gamepad2 to move it down. As soon as it is released, it will go back to its default position.
 * Press the b button on gamepad2 to toggle the braking mechanism on the linear slide in case it is coasting while hooked on to the lander.
 */

@TeleOp(name = "Basic TeleOp Linear OpMode for 2018-2019 Season", group = "Linear Opmode")
public class Working12820TeleOp extends LinearOpMode {

    // Declare Timer and Hardware Variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor linearSlide = null;
    private DcMotor intake = null;
    private Servo frontServo = null;
    private ColorSensor cSensor = null;
    private DistanceSensor dSensor = null;

    // Sets up a variables to control power and position of motors and servos
    double leftPower = 0;
    double rightPower = 0;
    double linearPower = 0;
    double intakePower = 0;
    double frontServoPosition = 0;

    //boolean for toggling brake
    boolean brakeOn = false;

    //set up variables for important positions of servo
    private static final double SERVO_DOWN = 0;
    private static final double SERVO_UP = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        // "deviceNames must correspond to the names assigned in the robot configuration file on the robot controller phone.
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");              //set up left motor location on robot
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");             //set up right motor location on robot
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");           //set up linear slide motor location on robot
        intake = hardwareMap.get(DcMotor.class, "arm_servo");                   //set up intake location on robot
        frontServo = hardwareMap.get(Servo.class, "front_servo");               //set up front servo location on robot
        cSensor = hardwareMap.get(ColorSensor.class, "color_sensor");           //set up color sensor location on robot
        dSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");     //set up distance sensor location on robot

        //Assign directions so that motors spin in the right direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            updateVars();
            updateSystem();
            updateT();
            //idle(); // no difference has been noted by the TSTEM TRILOBYTES Software Team regarding idle()
        }
    }

    /**
     * Update power and position variables for motors and servos
     */
    public void updateVars() {
        // Update Tank Drive variables
        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        //Update linear slide power variable
        if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 0) {
            linearPower = 1;
        } else if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 1) {
            linearPower = -1;
        } else {
            linearPower = 0;
        }

        //Intake power
        intakePower = -gamepad2.left_stick_y;

        //Front servo position
        if (gamepad2.y)
            frontServoPosition = SERVO_DOWN;
        else
            frontServoPosition = SERVO_UP;

    }

    /**
     * This method toggles between BRAKE mode and FLOAT mode on the linear slide motor for convenience
     * during teleOp
     */
    public void setBrake() {
        if (!brakeOn) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            brakeOn = true;
        } else {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            brakeOn = false;
        }
    }

    /**
     * This method sets the hardware components to the updated variables.
     */
    public void updateSystem() {
        // Send calculated power to respective robot systems
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        linearSlide.setPower(linearPower);
        intake.setPower(intakePower);

        frontServo.setPosition(frontServoPosition);
        if (gamepad2.b)
            setBrake();
    }

    /**
     * This method provides sensory information and time for convenience during gameplay and for debugging purposes
     * See overloaded method below
     */
    public void updateT() {
        // Show the elapsed game time, wheel power, linear slide power, arm motor power, servo position, and sensory data.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left expected: (%.2f), left power: (%.2f), right expected: (%.2f), right power: (%.2f)", leftPower, leftDrive.getPower(), rightPower, rightDrive.getPower());
        telemetry.addData("Arm Motors", "linear slide expected: (%.2f), linear slide power: (%.2f), intake expected: (%.2f), intake power: (%.2f)", linearPower, linearSlide.getPower(), intakePower, intake.getPower());
        telemetry.addData("Servos", "front servo expected position: (%.2f), front servo actual position: (%.2f)", frontServoPosition, frontServo.getPosition());
        telemetry.addData("Sensors", "distance sensor: (%.2f)", dSensor.getDistance(DistanceUnit.METER));
        telemetry.addData("Sensors", "color sensor: red:(%d) green:(%d) blue:(%d) alpha:(%d) argb:(%d) ", cSensor.red(), cSensor.green(), cSensor.blue(), cSensor.alpha(), cSensor.argb());
        telemetry.update();
    }

    /**
     * Overloaded method for above updateT().
     * This method takes a String named input and adds it to the telemetry.
     * It then updates the telemetry on the driver station.
     *
     * @param input
     */
    public void updateT(String input) {
        telemetry.addLine(input);
        telemetry.update();
    }
}
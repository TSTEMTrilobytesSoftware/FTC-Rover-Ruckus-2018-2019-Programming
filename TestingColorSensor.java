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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * @author 12820 Wagner High School FTC Robotics Programming Team: Ian Fernandes, Chloe Price, Aden Briano, Bella Gonzalez, Ruby Gomez, Daniel Coronado
 * @version 1.0
 * This program corresponds to the Autonomous portion of the FTC match.
 * This program was designed to test the color sensor functionalities. This will allow the programmer
 * to gain more insight on how the color sensor works and then use that to implement it into the
 * main autonomous program that uses a color sensor.
 */

@Autonomous(name = "Testing Color Sensor", group = "Linear Opmode")
public class TestingColorSensor extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor linearSlide = null;
    private CRServo armServo = null;
    private Servo frontServo = null;
    private DistanceSensor dSensor = null;
    private ColorSensor cSensor = null;

    // Declare variables for driver station telemetry.
    private double leftPower = 0;
    private double rightPower = 0;
    private double linearPower = 0;
    private double armServoPosition = 0;
    private double frontServoPosition = 0;

    // Wait till initialization...
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
        dSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        cSensor = hardwareMap.get (ColorSensor.class, "color_sensor");


        // It is typical for most robots to need one of the motors reversed in order to allow for proper operation.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);

        // Set the linear slide zero power behavior to BRAKE so that it does not coast when hanging onto the lander.
        linearSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //Create a timer so that just in case the robot doesn't sense the mineral, it will still continue with the rest of the program.
        ElapsedTime hey = new ElapsedTime();
        hey.reset();
        hey.startTime();
        //75-90 65-80, 30-40

        //Keep driving forward until the robot "sees" RGB values in the specified range which should amount to yellow.
        while(!(cSensor.red()>=75&&cSensor.red()<=90&&cSensor.green()>=65&&cSensor.green()<=80&&cSensor.blue()>=30&&cSensor.blue()<=40)&&hey.seconds()<5){
            leftDrive.setPower(.3);
            rightDrive.setPower(.3);
            updateT();
        }

        //reset timer and motor powers when yellow object is detected.
        hey.reset();
        updateT();
        leftDrive.setPower(0);
        rightDrive.setPower(0);


    }

    /**
     * Update the telemetry of the driver station based on data from motors, servos, and sensors.
     */
    public void updateT() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left: (%.2f), right: (%.2f), linear slide: (%.2f)", leftPower, rightPower, linearPower);
        telemetry.addData("Arm Motors", "linear slide: (%.2f)", linearPower);
        telemetry.addData("Servos", "arm servo: (%.2f), front servo: (%.2f)", armServoPosition, frontServoPosition);
        telemetry.addData("Sensors", "distance sensor: (%.2f)" , dSensor.getDistance(DistanceUnit.METER));
        telemetry.addData("Sensors", "color sensor: red:(%d) green:(%d) blue:(%d) alpha:(%d) argb:(%d) ", cSensor.red(),cSensor.green(),cSensor.blue(),cSensor.alpha(),cSensor.argb());
        telemetry.update();
    }

}

/* Copyright (c) 2018 FIRST. All rights reserved.
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

//import necessary classes to run OpMode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

/**
 * @author 12820 Wagner High School FTC Robotics Programming Team: Ian Fernandes, Aden Briano, Bella Gonzalez, Daniel Coronado
 * @version 2.0
 * This program corresponds to the Autonomous portion of the FTC match.
 * This OpMode uses Tensorflow through the instantiation of a Vuforia engine to sample.
 * It then proceeds to complete the rest of the autonomous routine.
 * To be used on LEFT side of lander for ROVER RUCKUS game.
 */
@Autonomous(name = "Left Auto", group = "Concept")
public class TensorFlowTest extends LinearOpMode {
    //variable for how well Tensorflow needs to detect the object to detect it right
    private static Double confidence;
    //other variables for Tensorflow obect detection
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final double SERVO_DOWN = 0.3;
    //set up hardware members for robot movement on the gamefield
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo frontServo = null;

    /**
     * variable that defines where the mineral is located at.
     * default position is -2. If it is still set to the default position after Tensorflow runs, this means that
     * Tensorflow could not detect the minerals correctly. Therefore, it will carry out the default tasks
     */
    int position = -2;


    /*
     *Vuforia License Key
     */
    private static final String VUFORIA_KEY = "AYeuFGH/////AAABmdqwAJ8Ig0PzslYt8z33d4MGbvGcowYMzAKjLykY+mRujQ4jYj5/XaRInlA0LVW6gAu/RG4TOnJMQljgRytE7Lo1aQOj6VUN+YWOacDSLvowzA3XiL0+5lKbbCJ9TZ3+q80c5TsKXMuWQWp907bas8qQt7tOZsk+YXvuQJpvsp5b9QYRy9/FAXVAxdG45NeXY/vuqUtXM/yxmXd5WdMoDRdgRt7H+0aQYjouyVvwlJSorzPnSZz7aCNpQiSdFg4As/qmBO/tt8xJ+81wCp1kC8TYhUqk9rEMcpypWs+TfUxqen3yoZwGDqw7OFF0va52enmBUh4nd3v6cz16Vd8SovUxxzG1uJO5y4y6Iw42D5dX";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */

    private TFObjectDetector tfod;
    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
    // first.

    @Override
    public void runOpMode() {

        //initialize variables to hardware locations on REV Expansion Hub.
        //rightDrive was set to a hardware location on REV Expansion Hub configuration file
        //as left_drive to fix coding logic. If we have time, we will fix the names
        //and switch the motor locations on the robot.
        rightDrive = hardwareMap.get(DcMotor.class, "left_drive");
        //leftDrive was set to a hardware location on REV Expansion Hub configuration file
        //as right_drive to fix coding logic. If we have time, we will fix the names
        //and switch the motor locations on the robot.
        leftDrive = hardwareMap.get(DcMotor.class, "right_drive");
        frontServo = hardwareMap.get(Servo.class, "front_servo");

        //set directions of the motors so that robot can properly move when given directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //set zero power behavior to BRAKE so that motors achieve greater accuracy when stopping and moving
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        //initialize Vuforia instance
        initVuforia();

        //make sure device is compatible with TFOD
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        //check to make sure op mode is still active
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            //CameraDevice.getInstance().setFlashTorchMode(true);

            //create new elapsed timer to time Tensorflow's execution
            ElapsedTime time = new ElapsedTime();
            time.reset();
            time.startTime();

            //allow 7 seconds for Tensorflow to detect objects. this ensures that if the object is
            //not found, the position is set to a value that we can later interpret as inproper
            //detection or no detection at all.
            while (time.seconds() < 7) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    //add data to telemetry for object detection statistics
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.addData("minimum confidence", confidence);
                        //update variables for future processing to determine position of gold mineral
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            //add telemetry data for position of gold mineral and update position variable
                            //so that position can be interpreted later
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    position = -1;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    position = 1;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    position = 0;
                                }
                                telemetry.addData("goldMineralX", goldMineralX);

                            }
                        }
                        telemetry.update();
                    }
                }
            }
            time.reset();
            /*
            Variable values for position:
            -1 = left
            0 = center
            1 = right
            -2 = Tensorflow faulty or incorrect detection
             */

            //if gold mineral position is center or if Tensorflow didn't detect objects, take forward
            //route to move middle mineral out of position
            if (position == -2 || position == 0) {
                if (position == -2) {
                    telemetry.addData("Status", "didn't detect");
                    telemetry.update();
                }

                //go forward
                leftDrive.setPower(.3);
                rightDrive.setPower(.3);
                sleep(3000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(500);

                //go backward
                leftDrive.setPower(-.3);
                rightDrive.setPower(-.3);
                sleep(2600);
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                //correct inaccuracy in drive system
                leftDrive.setPower(.3);
                sleep(400);
                leftDrive.setPower(0);


                //if position ==-1, the mineral is on the left. Turn left and move gold mineral
                //out of position before returning to starting position
            } else if (position == -1) {
                //turn slightly left
                rightDrive.setPower(.3);
                sleep(2500);
                rightDrive.setPower(0);

                //go forward
                leftDrive.setPower(.3);
                rightDrive.setPower(.3);
                sleep(3000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(500);

                //go backward
                leftDrive.setPower(-.3);
                rightDrive.setPower(-.3);
                sleep(2900);
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                //turn back to original position
                rightDrive.setPower(-.3);
                sleep(2150);
                rightDrive.setPower(0);
            }
            //if position == 1, the mineral is on the right. Turn right and move gold ineral
            //out of position before retrning to starting position
            else if (position == 1) {
                //turn slightly right
                leftDrive.setPower(.3);
                sleep(2500);
                leftDrive.setPower(0);

                //go straight
                leftDrive.setPower(.3);
                rightDrive.setPower(.3);
                sleep(3000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(500);

                //go backward
                leftDrive.setPower(-.3);
                rightDrive.setPower(-.3);
                sleep(2900);
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                //turn back to original position
                leftDrive.setPower(-.3);
                sleep(2150);
                leftDrive.setPower(0);
            }

            //The following code should run regardless of what happens above. In other words,
            //after the mineral has been moved, the robot will continue with the rest of the
            //autonomous program.

            //turn left
            rightDrive.setPower(1);
            sleep(850);
            rightDrive.setPower(0);

            sleep(100);

            //go forward
            rightDrive.setPower(1);
            leftDrive.setPower(1);
            sleep(1100);
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            //turn left
            leftDrive.setPower(1);
            sleep(1200);
            leftDrive.setPower(0);

            sleep(100);

            //go straight
            leftDrive.setPower(1);
            rightDrive.setPower(1);
            sleep(1800);
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            //drop of team marker with frontServo
            setFrontServo(SERVO_DOWN);
            sleep(3000);

            //correct possible misalignment on field
            rightDrive.setPower(-1);
            sleep(100);
            rightDrive.setPower(0);

            sleep(100);

            //reverse to crater
            leftDrive.setPower(-1);
            rightDrive.setPower(-1);
            sleep(5000);
            leftDrive.setPower(0);
            rightDrive.setPower(0);


        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minimumConfidence = 0.75;
        confidence = tfodParameters.minimumConfidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * setFrontServo sets the frontServo, which handles the team marker, to the position
     * specified as the parameter.
     *
     * @param position
     */
    public void setFrontServo(double position) {
        if (position >= 0 && position <= 1)
            frontServo.setPosition(position);
        else
            frontServo.setPosition(0);
    }
}

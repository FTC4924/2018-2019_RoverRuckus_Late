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

package org.firstinspires.ftc.teamcode.Old;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlowFront Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Marker Starting World", group = "4924")

public class DeliverMarkerSpecial extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor linearMotor;
    TouchSensor limitSwitch;
    Servo linearServo;
    // CRServo collectionServo;
    Servo marker;
    CRServo tape;
    CRServo tapeM;
    Servo led;
    Servo mineralServo;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    DcMotor extension;
    DcMotor rotation;
    Servo armBump;


    static final double     COUNTS_PER_MOTOR_REV    = 1425.2 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.75;

    static BNO055IMU imu;

    static BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
    Orientation angles;
    protected static DcMotor[] DRIVE_BASE_MOTORS = new DcMotor[4];
    private static final double GYRO_TURN_TOLERANCE_DEGREES = 3;
    boolean rotationOut = false;
    boolean landed = false;
    boolean latched = false;
    boolean kicked = false;
    int direction = 0;
    boolean detected = false;
    boolean color = false;
    boolean done1 = false;
    boolean done2 = false;
    int goldPosition;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        linearServo = hardwareMap.get(Servo.class, "linearServo");
        //collectionServo = hardwareMap.get(CRServo.class, "collectionServo");
        marker = hardwareMap.get(Servo.class, "markerServo");
        tape = hardwareMap.get(CRServo.class, "tapeMeasure");
        tapeM = hardwareMap.get(CRServo.class, "tapeServo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        led = hardwareMap.get(Servo.class, "led");
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");
        extension = hardwareMap.get(DcMotor.class, "extension");
        rotation = hardwareMap.get(DcMotor.class, "rotation");
        mineralServo = hardwareMap.get(Servo.class, "mineralServo");
        armBump = hardwareMap.get(Servo.class, "armBump");

        armBump.setPosition(.45);
        led.setPosition(0.7745);
        mineralServo.setPosition(0.45);

        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DRIVE_BASE_MOTORS[0] = frontLeft;
        DRIVE_BASE_MOTORS[1] = frontRight;
        DRIVE_BASE_MOTORS[2] = backLeft;
        DRIVE_BASE_MOTORS[3] = backRight;
        setMotorsModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER, DRIVE_BASE_MOTORS);
        setMotorsModes(DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_BASE_MOTORS);

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status:", "Starting");
        telemetry.update();


        while (opModeIsActive()) {
            if (!landed && !latched) {
                runtime.reset();
                if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                    initTfod();
                } else {
                    telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                }
                /** Activate Tensor Flow Object Detection. */
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }
                while (opModeIsActive() && !detected) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
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
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1 && !kicked) {
                                   /*If gold is on the left then go forward 2 inches then turn left and go forward 14
                                    inches then turn to face depot and drop marker then face crater on other color side
                                    then drive forward and set power to tape.*/
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        direction = -1;
                                        detected = true;

                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                       /*If gold is on the right then go forward 2 inches then turn right and go forward 12
                                    inches then turn to face depot and go 12 inches and then drop marker then face crater on
                                    other color side then drive forward and set power to tape.*/

                                        direction = 1;
                                        detected = true;
                                    } else {
                                        /*If gold is in the center then go forward 12 inches and drop marker then go backwards
                                        and face crater on other color side then drive forward and set power to tape.*/
                                        direction = 0;
                                        detected = true;
                                    }

                                }
                            } else if (runtime.seconds() >= 5 && !kicked) {
                                //It has been 20 seconds and we cannot identify the gold
                                //Assume middle
                                direction = 0;
                                detected = true;
                            }
                            telemetry.update();
                        }
                    }
                }
                if (tfod != null) {
                    tfod.shutdown();
                }
                linearServo.scaleRange(0.0, 1.0);
                linearMotor.setTargetPosition(-7122);
                linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearMotor.setPower(1);
                if (linearMotor.getCurrentPosition() < -7100) {
                    landed = true;
                    telemetry.addData("Status:", "Landed");
                    telemetry.update();
                }
            }
            if (landed && !latched) {
                linearServo.setPosition(1);
                // collectionServo.setPower(1);
                sleep(2000);
                //collectionServo.setPower(0);
                latched = true;
                telemetry.addData("Status:", "Latched");
                telemetry.update();
            }
            if (landed && latched && !color) {

                if (direction == -1) {
                    kicked = true;
                    telemetry.addData("Gold Mineral Position", "Left");
                    encoderDrive(DRIVE_SPEED, 2, 2, 5);
                    turnToPosition(.5, 25);
                    encoderDrive(DRIVE_SPEED, 14, 14, 5);
                    turnToPosition(.5, -45);
                    encoderDrive(DRIVE_SPEED, 5, 5, 5);
                    marker.setPosition(0);
                    sleep(1300);
                    marker.setPosition(75);
                    turnToPosition(.5, -45);
                    encoderDrive(DRIVE_SPEED, -10, -10, 5);
                    turnToPosition(.5, -50);
                    color = true;

                } else if (direction == 1) {
                                       /*If gold is on the right then go forward 2 inches then turn right and go forward 12
                                    inches then turn to face depot and go 12 inches and then drop marker then face crater on
                                    other color side then drive forward and set power to tape.*/
                    kicked = true;
                    telemetry.addData("Gold Mineral Position", "Right");
                    encoderDrive(DRIVE_SPEED, 2, 2, 5);
                    turnToPosition(.5, -25);
                    encoderDrive(DRIVE_SPEED, 12, 12, 5);
                    turnToPosition(.5, 25);
                    encoderDrive(.5, 9, 9, 5);
                    marker.setPosition(0);
                    sleep(1300);
                    marker.setPosition(75);
                    encoderDrive(DRIVE_SPEED, 3, 3, 5);
                    turnToPosition(.5, -45);
                    encoderDrive(DRIVE_SPEED, -10, -10, 5);
                    color = true;

                } else {
                                        /*If gold is in the center then go forward 12 inches and drop marker then go backwards
                                        and face crater on other color side then drive forward and set power to tape.*/
                    kicked = true;
                    encoderDrive(DRIVE_SPEED, 2, 2, 5);
                    telemetry.addData("Gold Mineral Position", "Center");
                    encoderDrive(DRIVE_SPEED, 10, 10, 5);
                    encoderDrive(.5, 5, 5, 5);
                    marker.setPosition(0);
                    sleep(1300);
                    marker.setPosition(75);
                    encoderDrive(DRIVE_SPEED, -12, -12, 5);
                    turnToPosition(0.5, -70);
                    encoderDrive(DRIVE_SPEED, -5, -5, 5);
                    turnToPosition(0.5, -75);
                    color = true;

                }
            }
            if (color && !done1) {
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);

               if (sensorColor.alpha() > 5500) {
                   telemetry.addData("Color", "White");
                   telemetry.addData("alpha", sensorColor.alpha());
                   tapeM.setPower(1);
                   tape.setPower(1);
               } else {
                   tape.setPower(0);
                   tapeM.setPower(0);
                   telemetry.addData("Tape", "Stop");
                   telemetry.addData("alpha", sensorColor.alpha());
                   done1 = true;
               }
               telemetry.update();
            }
            telemetry.update();

            if (color && !done2){
                armBump.setPosition(0);
                rotation.setTargetPosition(1322);
                rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotation.setPower(1);
                if(!rotationOut) {
                    if (rotation.getCurrentPosition() > 300) {
                        rotation.setPower(0);
                        extension.setPower(-1);
                        sleep(1400);
                        extension.setPower(0);
                        rotationOut=true;

                    }
                }

                if (rotationOut){
                    rotation.setPower(1);
                    if (rotation.getCurrentPosition() > 1300) {
                        rotation.setPower(0);
                        extension.setPower(-1);
                        if (limitSwitch.isPressed()) {
                            extension.setPower(0);
                            done2 = true;
                        }
                    }
                }

            }

            if(done1 && done2){
                sleep(10000);
            }
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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newLeftTarget);
            backLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);
            backRight.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to " + newLeftTarget + newRightTarget);
                telemetry.addData("Path2",  "Running at " +
                        frontLeft.getCurrentPosition() +
                        frontRight.getCurrentPosition());
                telemetry.addData("driving",leftInches);

                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turn(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void turnToPosition(double turnPower, double desiredHeading) {

        setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DRIVE_BASE_MOTORS);

        while (getHeading() - desiredHeading > GYRO_TURN_TOLERANCE_DEGREES && opModeIsActive()) {
            telemetry.addData("Heading", getHeading());
            telemetry.update();
            turn(turnPower);
        }
        while (getHeading() - desiredHeading < -GYRO_TURN_TOLERANCE_DEGREES && opModeIsActive()) {
            telemetry.addData("Heading", getHeading());
            telemetry.update();
            turn(-turnPower);
        }
        setMotorsPowers(0, DRIVE_BASE_MOTORS);
        //defaults to CW turning
    }

    public double getHeading() {
        updateGyro();
        telemetry.addData("getHeading",angles.firstAngle);
        telemetry.update();
        return angles.firstAngle;
    }

    void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public static void setMotorsModes(DcMotor.RunMode runMode, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setMode(runMode);
    }

    protected static void setMotorsPowers(double power, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setPower(power);
    }
}
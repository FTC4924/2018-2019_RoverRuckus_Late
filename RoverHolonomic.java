
package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "RoverHolonomic", group = "Iterative Opmode")

@Disabled
public class RoverHolonomic extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor collection = null;
    private DcMotor linearMotor = null;
    private Servo linearServo = null;
    private CRServo collectionServo = null;
    private CRServo armServo = null;
    private CRServo tape = null;
    private Servo rightArm = null;
    private CRServo marker = null;
    private CRServo tapeM = null;
    private Servo tapeBump = null;
    private Servo deliveryServo = null;
    double clawClosePosition = 0.5;
    /*

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        final double MIDDLEPOSITION180 = 0.0;
        //final double MIDDLE_POSITION_CONTINOUS = 0.0;
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        collection = hardwareMap.get(DcMotor.class, "collection");
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        collectionServo = hardwareMap.get(CRServo.class, "collectionServo");
        linearServo = hardwareMap.get(Servo.class, "linearServo");
        armServo = hardwareMap.get(CRServo.class, "armServo");
        tape = hardwareMap.get(CRServo.class, "tapeMeasure");
        tapeM = hardwareMap.get(CRServo.class, "tapeServo");
        tapeBump = hardwareMap.get(Servo.class, "tapeBump");
        deliveryServo = hardwareMap.get(Servo.class, "deliveryServo");
        marker = hardwareMap.get(CRServo.class,"markerServo");
        //rightArm = hardwareMap.get(Servo.class, "rightArm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        collection.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);

        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        linearServo.scaleRange(0.0, 1.0);

        double collectionPower = 0.0;
        double sideways = 0.0;
        double extendCollection = 0.0;
        boolean extended = true;
        double deliveryPower = 0.0;
        boolean barDownPosition;
        final double BARMOVE = 1.0;
        final double BARCLOSE = 0.0;
        boolean elbowBent;
        double position = 0.0;
        double clawPosition = 0.0;

        //we set what to do when the motor is not given power, which is to brake completely, instead of coasting
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double drive = -gamepad1.left_stick_y;
        //drive is what direction we want to move, either forwards, backwards, or neither
        double holonomic = -gamepad1.left_stick_x;
        //holonomic is what direction we want to move sideways
        double turnRight = gamepad1.right_trigger;
        //turnRight is how much we want to turn right
        double turnLeft = gamepad1.left_trigger;
        //turnLeft is how much we want to turn left
        boolean collectionPowerUp = gamepad2.b;
        //collectionPowerUp is dependent on whether or not we want the collection to collect
        boolean collectionPowerDown = gamepad2.a;
        //collectionPowerDown is dependent on whether or not we want the collection deliver (Push downwards)

        double leadScrew =  0.5 * (gamepad2.left_stick_y);

        double tapeMeasure = -0.5 * (gamepad2.right_stick_y);

        boolean collectionUp = gamepad2.y;
        boolean collectionDown = gamepad2.x;

        boolean halfSpeed = gamepad1.left_bumper;

        if (gamepad2.dpad_up) {
            //if we want it to collect, we set collectionPower to 1
            leadScrew = 0.5;

        } else if (gamepad2.dpad_down) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            leadScrew = -0.5;
        }

        if (gamepad2.left_trigger>0.01) {
            tapeBump.setPosition(.45);

        } else if (gamepad2.right_trigger>0.01) {
            tapeBump.setPosition(0);
        }

        if (collectionPowerUp) {
            //if we want it to collect, we set collectionPower to 1
            collectionPower = 1;
        } else if (collectionPowerDown) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            collectionPower = -1;
        }

        if (gamepad2.left_bumper) {
            //if we want it to collect, we set collectionPower to 1
            linearServo.setPosition(0.3);

        } else if (gamepad2.right_bumper) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            linearServo.setPosition(0.7);
        }

        if (gamepad1.a ) {
            //if we want it to collect, we set collectionPower to 1
            armServo.setPower(-0.6);
        } else if (gamepad1.b) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            armServo.setPower(0.6);
        } else{
            armServo.setPower(0);
        }

        if (gamepad1.x) {
            deliveryServo.setPosition(.45);

        } else if (gamepad1.y) {
            deliveryServo.setPosition(0);
        }


        if (gamepad1.dpad_up){
            marker.setPower(-0.5);
        } else if (gamepad1.dpad_down){
            marker.setPower(0.5);
        } else{
            marker.setPower(0);
        }

        if (collectionUp ) {
            //if we want it to c    ollect, we set collectionPower to 1
            extendCollection =  1;
        } else if (collectionDown) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            extendCollection = -1;
        }

       /* if (gamepad1.x){
            rightArm.setPosition(0.3);
        } else if(gamepad1.y){
            rightArm.setPosition(0.7);
        }*/



        //we are calculating the power to send to each different wheel, which each need their own power since it is calculated in different ways

        double frontLeftPower =  Range.clip(drive - holonomic + turnRight - turnLeft, -1.0, 1.0);
        double frontRightPower =  Range.clip(drive + holonomic - turnRight + turnLeft, -1.0, 1.0);
        double backRightPower =  Range.clip(drive - holonomic - turnRight + turnLeft, -1.0, 1.0);
        double backLeftPower =  Range.clip(drive + holonomic + turnRight - turnLeft, -1.0, 1.0);

        if (halfSpeed) {
            frontLeftPower = 0.35 * (frontLeftPower);
            frontRightPower = 0.35 * (frontRightPower);
            backRightPower = 0.35 * (backRightPower);
            backLeftPower = 0.35 * (backLeftPower);

        }

        // Send calculated power to wheels and motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        collection.setPower(collectionPower);
        linearMotor.setPower(leadScrew);
        collectionServo.setPower(extendCollection);
        tape.setPower(tapeMeasure);
        tapeM.setPower(tapeMeasure);

        // Show the elapsed game time
        // telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Slow Mode", halfSpeed);

        telemetry.addData("Elbow Servo", "Continous Position" + position);
        telemetry.addData("Lead Screw Motor", linearMotor.getCurrentPosition());
        telemetry.addData("Tape Measure", tapeMeasure);

        turnLeft = 0;
        turnRight = 0;

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "robotYay", group="Axolotl")
//@disabled so robot runs yay
public class robotYay extends OpMode {

    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR;
    private DcMotorEx flyL, flyR;
    private DcMotorEx ascendL, ascendR;
    private Servo pusher;


    // Position of pusher servo
    private double pusherSpeed = 0.02;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");

        //Movement Wheels Initialization
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        //Fly Wheels for launching initialization
        flyL = hardwareMap.get(DcMotorEx.class, "flyL");
        flyR = hardwareMap.get(DcMotorEx.class, "flyR");

        //Making sure that the flywheel direction for each motor is constant
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flyR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Motors for ascension during parking initialization
        ascendL = hardwareMap.get(DcMotorEx.class, "ascendL");
        ascendR = hardwareMap.get(DcMotorEx.class, "ascendR");

        // Motors for slide positions and all that
        ascendL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascendL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascendL.setDirection(DcMotorSimple.Direction.REVERSE);
        ascendL.setTargetPositionTolerance(20);
        ascendL.setTargetPosition(50);
        ascendL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ascendL.setVelocity(800);
        ascendL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ascendR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascendR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascendR.setDirection(DcMotorSimple.Direction.FORWARD);
        ascendR.setTargetPositionTolerance(20);
        ascendR.setTargetPosition(50);
        ascendR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ascendR.setVelocity(800);
        ascendR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ramp Servo Init
        pusher = hardwareMap.get(Servo.class, "pusher");


        telemetry.addData("Status", "Initialized and Ready");
    }

    @Override
    // Methods that will be initialized and be forever running during teleop
    public void loop() {
        drive();
        launchSystem();
        ascendParking();
        pusherMove();
    }

    public void drive() {
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;

        double speedMod = 0.5;
        double FL = (-x - y - rotation) * speedMod;
        double FR = (-x + y - rotation) * speedMod;
        double BL = ( x - y - rotation) * speedMod;
        double BR = ( x + y - rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }

    public void launchSystem() {
        double triggerValue = gamepad1.right_trigger;

        if (triggerValue > 0.1) {
            flyL.setPower(1);
            flyR.setPower(1);
        } else {
            flyL.setPower(0);
            flyR.setPower(0);
        }

        telemetry.addData("Flywheel Power", triggerValue);
        telemetry.update();
    }

    public void ascendParking(){

        int slidePos = 1500*18/14;

        if (gamepad1.y) {
            ascendL.setDirection(DcMotorSimple.Direction.FORWARD);
            ascendL.setTargetPosition(slidePos);
            ascendL.setVelocity(1000);
            ascendL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascendL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ascendR.setDirection(DcMotorSimple.Direction.REVERSE);
            ascendR.setTargetPosition(slidePos);
            ascendR.setVelocity(1000);
            ascendR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascendR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Slide", "Ascending");
            telemetry.update();
        }

        if (gamepad1.a) {
            ascendL.setDirection(DcMotorSimple.Direction.FORWARD);
            ascendL.setTargetPosition(0);
            ascendL.setVelocity(500);
            ascendL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascendL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ascendR.setDirection(DcMotorSimple.Direction.REVERSE);
            ascendR.setTargetPosition(0);
            ascendR.setVelocity(500);
            ascendR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascendR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Slide","Descending");
            telemetry.update();
        }
    }

    public void pusherMove() {

        if (gamepad1.right_bumper) {
            pusher.setPosition(pusherSpeed);
        } else if (gamepad1.left_bumper) {
            //pusherSpeed -= rampHeightSpeed;
            //ramp.setPosition(rampHeight);
        }
        telemetry.addData("Pusher Value", pusherSpeed);
        telemetry.update();
    }
}
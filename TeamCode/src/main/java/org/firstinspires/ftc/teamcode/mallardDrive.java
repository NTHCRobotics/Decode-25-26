package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "mallardDrive", group = "Axolotl")
public class mallardDrive {

    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR;
    private Servo elevate, rotate;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");

        //Movement Wheels Initialization
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        elevate = hardwareMap.get(Servo.class, "elevate");
        rotate = hardwareMap.get(Servo.class, "rotate");

        telemetry.addData("Status", "Initialized and Ready");
    }

    @Override
    // Methods that will be initialized and be forever running during teleop
    public void loop() {
        drive();
        ariseElevator();
        rotationManual();
        //launch();
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
    public void ariseElevator() {
        // ascendButton = gamepad1.y;

        if (gamepad1.y){
            elevate.setPosition(0.5);
        } else {
            elevate.setPosition(0.0);
        }

    }
    public void rotationManual(){
        // rotateButton = gamepad1.b;

        if (gamepad1.b){
            rotate.setPosition(0.5);
        } else {
            rotate.setPosition(0.0);
        }
    }
    public void launch(){
        // launchTrigger = gamepad1.right_trigger;

    }

}

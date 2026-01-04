package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "mallardDrive", group = "Axolotl")
public class mallardDrive extends OpMode {

    //Setting variables for motors and servos
    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR;
    private Servo elevate, rotate;
    int readings;
    private DistanceSensor distanceSensorFront, distanceSensorBack;

    NormalizedColorSensor colorSensorLeft, colorSensorRight;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");

        //Movement wheels initialization
        //   wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        //   wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        //   wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        //   wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        //Magazine servos initialization
        //    elevate = hardwareMap.get(Servo.class, "elevate");
        //    rotate = hardwareMap.get(Servo.class, "rotate");

        //Distance sensor initialization
        //distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceSensorFront");
        //distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");

        //Color sensor initialization
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        colorSensorLeft.setGain(35);

        colorSensorRight  = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");
        colorSensorRight.setGain(35);

        telemetry.addData("Status", "Initialized and Ready");

    }

    // loop variables
    int distanceCycle = 0;
    double distanceFront;
    double distanceBack;
    double[] arrayFront = new double[10];
    double[] arrayBack = new double[10];

    @Override
    //Methods that will be called and be forever running during teleop
    public void loop() {
        // drive();
        // ariseElevator();
        // rotationManual();
        //avgDistances();
        getDetectedColor();
        getDetectedColorRight();
        //rotationAutomatic();
        //launch();
    }

    //Methods
    public void drive() {
        //Setting variables for x and y (and rotation) axis positions on controller sticks
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;

        //Speed that motors run (half the speed they would originally run)
        double speedMod = 0.5;
        //Variables that calculate the speed each wheel goes (allows for turning and strafing)
        double FL = ( x + y + rotation) * speedMod;
        double FR = ( x - y + rotation) * speedMod;
        double BL = ( x - y - rotation) * speedMod;
        double BR = ( -x - y + rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }
    public void ariseElevator() {
        // ascendButton = gamepad1.y;
        // descendButton = gamepad1.a;

        if (gamepad1.y) {
            elevate.setPosition(0.1);
        } else if (gamepad1.a) {
            elevate.setPosition(0.9);
        } else {
            elevate.setPosition(0.5);
        }
    }


    public void avgDistances()
    {
        DistanceSensor dsb = distanceSensorBack;
        DistanceSensor dsf = distanceSensorFront;

        arrayFront[distanceCycle] = dsf.getDistance(DistanceUnit.MM);
        arrayBack[distanceCycle] = dsb.getDistance(DistanceUnit.MM);

        distanceCycle++;

        if (distanceCycle > 9) { distanceCycle = 0; }

        distanceFront = getAvg(arrayFront);
        distanceBack = getAvg(arrayBack);

        telemetry.addData("Distance Front: ", distanceFront + "mm");
        telemetry.addData("Distance Back: ", distanceBack + "mm");
        telemetry.addData("Distance no algorithm Front: ", distanceSensorFront.getDistance(DistanceUnit.MM) + "mm");
        telemetry.addData("Distance no algorithm Back: ", distanceSensorBack.getDistance(DistanceUnit.MM) + "mm");

    }

    public double getAvg(double[] list)
    {
        int sum = 0;
        for (int i = 0; i < list.length; i++)
        {
            sum += list[i];
        }
        return sum / list.length;
    }
    public void rotationManual(){
        // rotateButton = gamepad1.b;

//        telemetry.addData("deviceName", distanceSensorFront.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", (distanceSensorBack));
//        telemetry.addData("deviceName", distanceSensorBack.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", distanceSensorBack.getDistance(DistanceUnit.MM)));

        if (gamepad1.b){
            rotate.setPosition(0.9);
        } else if (distanceSensorFront.getDistance(DistanceUnit.MM) > 0) {
            rotate.setPosition(0.0);
        }
    }
    public void launch(){

        // launchTrigger = gamepad1.right_trigger;

    }
    public void getDetectedColor(){
        //return 4 values in percentages (red, green, blue, alpha(brightness))
        NormalizedRGBA colors = colorSensorLeft.getNormalizedColors();
        //colors = colorSensorRight.getNormalizedColors();

        float leftRed, leftGreen, leftBlue;
        leftRed = colors.red / colors.alpha;
        leftGreen = colors.green / colors.alpha;
        leftBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", leftRed);
        telemetry.addData("Green", leftGreen);
        telemetry.addData("Blue", leftBlue);

        // purple = .2142, .2683, .366
        // green = .1275, .4153, .3023

        if (leftRed > .20 && leftGreen < .27 && leftBlue > .36){
            telemetry.addData("Color detected LEFT", "PURPLE");
        } else if (leftRed < .13 && leftGreen > .4 && leftBlue < .34){
            telemetry.addData("Color detected LEFT", "GREEN");
        } else {
            telemetry.addData("Color detected LEFT", "NONE");
        }

    }

    public void getDetectedColorRight(){
        //return 4 values in percentages (red, green, blue, alpha(brightness))
        NormalizedRGBA colors = colorSensorRight.getNormalizedColors();
        //colors = colorSensorRight.getNormalizedColors();

        float rightRed, rightGreen, rightBlue;
        rightRed = colors.red / colors.alpha;
        rightGreen = colors.green / colors.alpha;
        rightBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", rightRed);
        telemetry.addData("Green", rightGreen);
        telemetry.addData("Blue", rightBlue);

        // purple = .2142, .2683, .366
        // green = .1275, .4153, .3023

        if (rightRed > .20 && rightGreen < .27 && rightBlue > .36){
            telemetry.addData("Color detected RIGHT", "PURPLE");
        } else if (rightRed < .13 && rightGreen > .4 && rightBlue < .34){
            telemetry.addData("Color detected RIGHT", "GREEN");
        } else {
            telemetry.addData("Color detected RIGHT", "NONE");
        }
    }

}

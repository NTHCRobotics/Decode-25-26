package org.firstinspires.ftc.teamcode;
//IMPORTS
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp(name = "mainDrive", group = "Axolotl")
public class mainDrive extends OpMode{
    //Hardware
    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR, intakeB; //Motors
    private Servo magazineServo, loadServo; //Servos
    private NormalizedColorSensor colorSensorL; //Left Color Sensor
    private NormalizedColorSensor colorSensorBR; //Right Back Color Sensor
    private NormalizedColorSensor colorSensorFR; //Right Front Color Sensor



    //Variables
    double speedMod = 0.5; //Speed of wheel motors (around 1/2 maximum rate)
    double[] magazineReadPositions = {0.0, 360.0/3/300, 360.0/3/300*2}; //lists out 1/3rd rotations of the magazine. The weird math is to normalize it to [0,1] given the servo's 300 degree range.
    // !IMPORTANT! artifactOrder is an ArrayList since it needs to be repeatedly scanned and edited.
    // 0 --> L, 1 --> BR, 2 --> FR
    ArrayList<String> magazineOrder = new ArrayList<String>(); //correlates to the artifact color currently in readPositions (e.g. the artifacts in the magazine)

    String[] obeliskOrder = {null, null, null}; //Correlates to the competition artifact sequence (read at init(), never changed)


    @Override
    public void init(){
        //Hardware
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        //Servos
        magazineServo = hardwareMap.get(Servo.class, "magazineServo");
        loadServo = hardwareMap.get(Servo.class, "loadServo");
        //Sensors
        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");
        colorSensorBR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBR");
        colorSensorFR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFR");

        //Initialization
        magazineServo.setPosition(0.0);
        loadServo.setPosition(0.0);

        //Initialize magazine orders will null (so size = 3)
        magazineOrder.add("empty");
        magazineOrder.add("empty");
        magazineOrder.add("empty");

    }

    @Override
    public void loop(){
        drive(); // translation and rotation
        //spinIntakes(); //Spinning the intakes (duh) DISABLED UNTIL BUTTON IS BOUND
        indexArtifacts(); // Keeps a running list of what artifacts exist within the magazine
        updateTelemetry(); // updates driver's hub telemetry
    }

    //Custom Classes
    //Read obelisk
    public void readObelisk() {
        String[] obeliskOrder = {"green", "purple", "green"}; //Temporary return while apriltag read is in progress
    }

    //Basic movement
    public void drive() {
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;

        //Wheel-by-wheel spin calculations
        double FL = (-x - y - rotation) * speedMod;
        double FR = (-x + y - rotation) * speedMod;
        double BL = (x - y - rotation) * speedMod;
        double BR = (x + y - rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }
    //Intake control
    public void spinIntakes() {
        intakeB.setPower(0.5);
    }

    //Color sensor
    public String colorMatch(NormalizedColorSensor colorSensor){
        double red = colorSensor.getNormalizedColors().red;
        double green = colorSensor.getNormalizedColors().green;
        double blue = colorSensor.getNormalizedColors().blue;

        double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
        double greenMatch = Math.max(0, green - Math.max(red, blue));

        double purpleTolerance = 0.5;
        double greenTolerance = 0.5;

        if (purpleMatch < purpleTolerance && greenMatch < greenTolerance){
            return "empty";
        }
        else if (purpleMatch > greenMatch){
            return "purple";
        }
        else {
            return "green";
        }
    }

    public void indexArtifacts() {
        String colorL = colorMatch(colorSensorL);
        String colorBR = colorMatch(colorSensorBR);
        String colorFR = colorMatch(colorSensorFR);

        magazineOrder.set(0, colorL);
        magazineOrder.set(1, colorBR);
        magazineOrder.set(2, colorFR);
    }

    public void launchInOrder() throws InterruptedException {
        for (int i = 0; i < 3; i++){
            int index = magazineOrder.indexOf(obeliskOrder[i]);
            magazineServo.setPosition(magazineReadPositions[index]);
            sleep(200); //Allows for the magazine to rotate
            //LAUNCH CODE HERE
            magazineOrder.set(i, "empty");
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Magazine Order", magazineOrder.get(0) + " " + magazineOrder.get(1) + " " + magazineOrder.get(2));
    }

}

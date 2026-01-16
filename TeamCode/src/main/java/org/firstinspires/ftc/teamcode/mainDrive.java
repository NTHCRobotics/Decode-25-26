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
    private NormalizedColorSensor colorSensor; //Sensors

    //Variables
    double speedMod = 0.5; //Speed of wheel motors (around 1/2 maximum rate)
    double[] magazineReadPositions = {0.0, 360.0/3/300, 360.0/3/300*2}; //lists out 1/3rd rotations of the magazine. The weird math is to normalize it to [0,1] given the servo's 300 degree range.
    // !IMPORTANT! artifactOrder is an ArrayList since it needs to be repeatedly scanned and edited.
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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");

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
    public double[] colorMatch(NormalizedColorSensor colorSensor){
        double red = colorSensor.getNormalizedColors().red;
        double green = colorSensor.getNormalizedColors().green;
        double blue = colorSensor.getNormalizedColors().blue;

        double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
        double greenMatch = Math.max(0, green - Math.max(red, blue));

        return new double[] {purpleMatch, greenMatch};
    }

    public void indexArtifacts() throws InterruptedException {
        for (int i = 0; i < 3; i++){
            magazineServo.setPosition(magazineReadPositions[i]);
            sleep(200); //Allows for the magazine to rotate
            double[] color = colorMatch(colorSensor);
            if (color[0] < 0.5 && color[1] < 0.5){
                magazineOrder.set(i, "empty");
            }
            else if (color[0] > color[1]){
                magazineOrder.set(i, "purple");
            }
            else {
                magazineOrder.set(i, "green");
            }
        }
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

    }

}

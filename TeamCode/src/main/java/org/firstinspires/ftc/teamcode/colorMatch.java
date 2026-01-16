package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@TeleOp()
public class colorMatch extends OpMode{
    //HARDWARE SETUP:
    //Color Sensors
    private NormalizedColorSensor colorSensor;

    private Servo magazineServo;

    //VARIABLES:
    //readPositions lists out 1/3rd rotations of the magazine. The weird math is to normalize it to [0,1] given the servo's 300 degree range.
    private double[] readPositions = {0.0, 360.0/3/300, 360.0/3/300*2};
    //artifactOrder correlated to the artifact color currently in readPositions. If it's empty or uninitialized, null is used.
    private String[] artifactOrder = {null, null, null};

    public void init(){

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");
        magazineServo = hardwareMap.get(Servo.class, "magazineServo");

        magazineServo.setPosition(0.0);

    }

    public void loop() {

    }

    public double[] colorPicker(NormalizedColorSensor colorSensor) {
        double red = colorSensor.getNormalizedColors().red;
        double blue = colorSensor.getNormalizedColors().blue;
        double green = colorSensor.getNormalizedColors().green;

        //[purplematch, greenmatch]

        double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
        double greenMatch = Math.max(0, green - Math.max(red, blue));

        return new double[]{purpleMatch, greenMatch};
    }

    public void indexArtifacts() throws InterruptedException {
        for (int i = 0; i < 3; i++){
            magazineServo.setPosition(readPositions[i]);
            sleep(1000);
            double[] color = colorPicker(colorSensor);
            if (color[0] < 0.5 && color[1] < 0.5){
                artifactOrder[i] = null;
            }
            else if (color[0] > color[1]){
                artifactOrder[i] = "purple";
            }
            else {
                artifactOrder[i] = "green";
            }

        }
    }

    public void updateTelemetry() {
        telemetry.addData("artifactOrder", artifactOrder[0] + " " + artifactOrder[1] + " " + artifactOrder[2]);
    }



}

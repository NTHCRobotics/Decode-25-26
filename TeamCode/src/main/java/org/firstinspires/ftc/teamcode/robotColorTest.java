package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.lang.Math;

@TeleOp()
public class robotColorTest extends OpMode{

    //Color Sensors
    private NormalizedColorSensor colorSensorL;

    public void init(){

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");


    }

    public void loop() {

    }

    public double[] colorMatch(NormalizedColorSensor colorSensor) {
        double red = colorSensor.getNormalizedColors().red;
        double blue = colorSensor.getNormalizedColors().blue;
        double green = colorSensor.getNormalizedColors().green;

        //[purplematch, greenmatch]

        double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
        double greenMatch = Math.max(0, green - Math.max(red, blue));

        return new double[]{purpleMatch, greenMatch};
    }




}

package org.firstinspires.ftc.teamcode;
//IMPORTS
import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp(name = "ATT", group = "Axolotl")
public class aprilTagTest extends OpMode{

    //April Tag + Vision Portal stuff
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    private String[] detection;

    private int aprilTagReadAttempts = 0;

    @Override
    public void init(){
        //Hardware
        //Motors

        //Initialize AprilTag Detection
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //Settings here
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();

    }

    @Override
    public void loop(){
        //spinIntakes(); //Spinning the intakes (duh) DISABLED UNTIL BUTTON IS BOUND
        try {
            detection = detectObelisk();
        } catch (InterruptedException e) {

        }
        updateTelemetry(); // updates driver's hub telemetry
    }

    public void updateTelemetry() {
        telemetry.addData("aprilTagReadAttempts", aprilTagReadAttempts);
        telemetry.addData("detection", detection[0] + " " + detection[1] + " " + detection[2]);
    }

    public String[] detectObelisk() throws InterruptedException {
        ArrayList<AprilTagDetection> obeliskDetection = aprilTagProcessor.getDetections(); //Gets the first detection of an apriltag, should only be the center one
        if (obeliskDetection.size() == 0 || obeliskDetection.get(0) == null) {
            if (aprilTagReadAttempts < 1000) { //100 is working maximum read attempts before giving up, total of 10 seconds of processing every 0.1 seconds.
                aprilTagReadAttempts++;
                sleep(100);
                return detectObelisk();
            }
            else {
                return new String[]{"undetected", "undetected", "undetected"}; //DEFAULT CATCH IN CASE OF EXCEPTIONS
            }
        }
        else {
            int id = obeliskDetection.get(0).id;
            switch (id) {
                case 21:
                    return new String[]{"green", "purple", "purple"};
                case 22:
                    return new String[]{"purple", "green", "purple"};
                case 23:
                    return new String[]{"purple", "purple", "green"};
                default:
                    return new String[]{"out of range", "out of range", "out of range"}; //DEFAULT CATCH IN CASE OF EXCEPTIONS
            }
        }

    }

}

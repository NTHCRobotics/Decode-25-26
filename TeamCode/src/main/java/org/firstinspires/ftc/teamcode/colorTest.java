package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class colorTest extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "colorSensorBR");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Guess", guessColorL(color.red(),color.green(),color.blue()));
            telemetry.update();
        }
    }

    public String guessColorL(int red, int green, int blue) {
        int total = red + green + blue;
        String answer = "";
        if (total > 400) {
            if ((green - blue > 15) && (green - blue > 0)) {
                answer = "green";
            }
            else if (blue - green < 15) {
                answer = "purple";
            } else {
                answer = "none";
            }
        }
        else
        {
            answer = "none";
        }
        return answer;
    }
}

package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by cafrostrobotics on 10/31/2017.
 */
@Autonomous(name="A:Relic Recovery", group="2017")
public class AutonomousAll extends OpMode {
    //

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeftFront = null;
    private DcMotor motorLeftRear= null;
    private DcMotor motorRightFront = null;
    private DcMotor motorRightRear = null;
    private DcMotor motorArm = null;
private String allianceColor = "blue";
    private Servo servoJewel = null;
    double servoPosition = 0;
    boolean isJewelServoDown = false;
    NormalizedColorSensor colorSensor;
    boolean isColorFound=false;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorLeftFront  = hardwareMap.get(DcMotor.class, "m1");
        motorLeftRear  = hardwareMap.get(DcMotor.class, "m2");
        motorRightFront = hardwareMap.get(DcMotor.class, "m3");
        motorRightRear = hardwareMap.get(DcMotor.class, "m4");
        motorArm = hardwareMap.get(DcMotor.class, "ma");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightRear.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        servoJewel = hardwareMap.get(Servo.class, "s3");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        String color = null;
        if (!isJewelServoDown) {
            servoJewel.setPosition(1);
        }
        if (!isColorFound){
             color = getColorSensorOutput();

        }
        else {
           if (color == allianceColor){
               //turn right

           }
          else {
               //turn left
           }
            //determine whether to turn left or right
        }

        /*
    * 1. Find Pictograph
    * 2.Read Pictograph (c,l,r)
    * 3. Move to jewels
    * 4. Identify colours
    * 5. Clear jewels.
    * 6. Go to cryptobox
    * 7. Find correct column
    * 8. Place glyph
    * 9. Get in cryptobox zone.
    * Glyph 2
    * 1. Find glyph
    * 2. Get glyph
    * 3. Identify colour
    * 4.Determine target column
    * 5. Goto cryptobox
    * 5. Find corre3ct column
    * 6. Place glyph
    */
    }
public String getColorSensorOutput(){
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
    boolean bPrevState = false;
    boolean bCurrState = false;

    bCurrState = gamepad1.x;

    // If the button state is different than what it was, then act
    if (bCurrState != bPrevState) {
        // If the button is (now) down, then toggle the light
        if (bCurrState) {
            if (colorSensor instanceof SwitchableLight) {
                SwitchableLight light = (SwitchableLight)colorSensor;
                light.enableLight(!light.isLightOn());
            }
        }
    }
    bPrevState = bCurrState;

    NormalizedRGBA colors = colorSensor.getNormalizedColors();
    Color.colorToHSV(colors.toColor(), hsvValues);
    telemetry.addLine()
            .addData("H", "%.3f", hsvValues[0])
            .addData("S", "%.3f", hsvValues[1])
            .addData("V", "%.3f", hsvValues[2]);
    telemetry.addLine()
            .addData("a", "%.3f", colors.alpha)
            .addData("r", "%.3f", colors.red)
            .addData("g", "%.3f", colors.green)
            .addData("b", "%.3f", colors.blue);
    int color = colors.toColor();
    telemetry.addLine("raw Android color: ")
            .addData("a", "%02x", Color.alpha(color))
            .addData("r", "%02x", Color.red(color))
            .addData("g", "%02x", Color.green(color))
            .addData("b", "%02x", Color.blue(color));

    float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
    colors.red   /= max;
    colors.green /= max;
    colors.blue  /= max;
    color = colors.toColor();

    telemetry.addLine("normalized color:  ")
            .addData("a", "%02x", Color.alpha(color))
            .addData("r", "%02x", Color.red(color))
            .addData("g", "%02x", Color.green(color))
            .addData("b", "%02x", Color.blue(color));
    telemetry.update();
    Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
    return "blue";
}
    @Override
    public void stop() {
    }
}

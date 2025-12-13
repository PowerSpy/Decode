package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class Vision {
    VisionPortal visionPortal;
    public Limelight3A limelight;
    private LLResult result = null;
    public double cameraAngle = Math.toRadians(15);
    public double cameraHeight = 10.0;
    public boolean obelisk = true;
    public int greenPosition = -1;

    int visionWidth = 480;
    int visionHeight = 360;

    public Vision (HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "limelight")) // i think this may work? need to test in the garage. I just want the video feed and limelight is registered as a camera
                .setCameraResolution(new Size(visionWidth, visionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        setCameraSettings(8, 145);
    }

    public void update(){
        if(!limelight.isConnected()){
            TelemetryUtil.packet.put("Limelight : Status", "Oops! Something broke :blehhh:");
        }else{
            result = limelight.getLatestResult();

            /*

            James let this be a reminder to never write important code past 1am - James 12/12/25

            if(obelisk && result != null && result.isValid()){
                // 21 = GPP
                greenPosition = (result.getFiducialResults().get(0).getFiducialId() - 21 + 2) % 3;
                startAprilTagDetection();
            }
            */
        }
    }

    public LLResult getResult(){ return result;}

    public boolean isDetected(){ return result != null && result.isValid(); }

    public void togglePipeline (boolean obelisk) {
        limelight.pipelineSwitch(obelisk ? 2 : (Globals.isRed ? 0 : 1));
        this.obelisk = obelisk;
    }

    // visionPortal methods

    public void startStreaming () {
        visionPortal.resumeStreaming();
    }

    public void stopStreaming () {
        visionPortal.stopStreaming();
    }

    public void setCameraSettings(int exposureVal, int gainVal) {
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(exposureVal, TimeUnit.MILLISECONDS);

        Log.e("exposure supported", exposure.isExposureSupported() + "");

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(gainVal);
    }
}

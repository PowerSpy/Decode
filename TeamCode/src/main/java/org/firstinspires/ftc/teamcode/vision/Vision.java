package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Vision {
    VisionPortal visionPortal;
    public Limelight3A limelight;
    private LLResult result = null;
    public static double cameraAngle = Math.toRadians(63.75);
    public static double cameraHeight = 12.75;
    public boolean obelisk = false;

    public Vision (HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(2);
    }

    public void update(){
        if(!limelight.isConnected()){
            TelemetryUtil.packet.put("Limelight : Status", "Oops! Something broke :blehhh:");
        }else{
            TelemetryUtil.packet.put("Limelight : Status", "at least it doesn't need glasses");
            result = limelight.getLatestResult();
        }
    }

    public void start() { limelight.start(); }

    public void stop() { limelight.stop(); }

    public LLResult getResult(){ return result;}

    public void setPipeline (int index) {
        limelight.pipelineSwitch(index);
        this.obelisk = index == 2;
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

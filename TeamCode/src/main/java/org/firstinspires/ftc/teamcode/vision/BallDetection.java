package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;

public class BallDetection {

    private Limelight3A limelight;

    private ArrayList<Vector2> ballPoses = new ArrayList<>();

    public static double stalenessThreshMs = 100;
    public static double confidenceThresh = 50;

    public BallDetection(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0); //Todo - set the correct pipeline
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid() && result.getStaleness() < stalenessThreshMs) {

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            ballPoses.clear();

            for (LLResultTypes.DetectorResult detection : detections) {
                if (detection.getConfidence() > confidenceThresh) {
                    double tx = detection.getTargetXDegrees();
                    double ty = detection.getTargetYDegrees();


                    //lowk in inches
                    double d = (Globals.LLHeight - Globals.ballRadius) / Math.tan(Math.toRadians(ty) + Math.toRadians(Globals.LlAngle));
                    //funny polar to cartesian conversion
                    Vector2 ballPos = new Vector2(ROBOT_POSITION.x + d * Math.cos(ROBOT_POSITION.heading + tx), ROBOT_POSITION.y + d * Math.sin(ROBOT_POSITION.heading + tx));

                    ballPoses.add(ballPos);
                } else {
                    Log.i("BallDetection", "Confidence too low");
                }
            }
        } else{
            Log.i("BallDetection", "Result is invalid");
        }

    }

    public ArrayList<Vector2> getBallPoses() {
        return ballPoses;
    }

}

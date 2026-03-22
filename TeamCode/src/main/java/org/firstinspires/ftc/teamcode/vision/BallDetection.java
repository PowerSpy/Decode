package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class BallDetection {
    private Limelight3A limelight;

    private ArrayList<Vector2> ballPoses = new ArrayList<>();

    public static double stalenessThreshMs = 100;
    public static double confidenceThresh = 0.7;

    public BallDetection(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(7);
    }

    public void start() {
        limelight.start();
        //TelemetryUtil.dashboard.startCameraStream(limelight, 30);
    }

    public void stop() {
        limelight.stop();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && result.getStaleness() < stalenessThreshMs) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            ballPoses.clear();

            for (LLResultTypes.DetectorResult detection : detections) {
                if (detection.getConfidence() > confidenceThresh) {
                    double tx = detection.getTargetXDegrees();
                    double ty = detection.getTargetYDegrees();
                    Log.i("Ty", String.valueOf(ty));

                    //lowk in inches
                    double d = (Globals.LLHeight - Globals.ballRadius) / Math.tan(Math.toRadians(ty + Globals.LlAngle));

                    //funny polar to cartesian conversion
                    double angle = ROBOT_POSITION.heading + Math.toRadians(tx);
                    Vector2 ballPos = new Vector2(ROBOT_POSITION.x + d * Math.cos(angle), ROBOT_POSITION.y + d * Math.sin(angle));
                    if (ballPos.x >= 0 && ballPos.y * (Globals.isRed ? 1 : -1) >= 0) {
                        ballPoses.add(ballPos);
                    }
                } else {
                    Log.i("BallDetection", "Confidence too low " + detection.getConfidence());
                }
            }
        } else{
            Log.i("BallDetection", "Result is invalid");
        }

        ballPoses.sort(Comparator.comparingDouble((Vector2 l) -> l.x));
    }

    public double[] getWeights(ArrayList<Vector2> ballPoses) {
        //list of criteria - we want the x difference between robot and ball to be as less as possible
        //we also weight the balls higher if there are balls near it in terms of x
        //weights are clamped between 1 and 0 with 1 being the perfect ball with there being 0 x differnce and two balls directly adjacent to it
        double[] weights = new double[ballPoses.size()];
        for(int i = 0; i < weights.length; i++) {
            double xDifference = Math.abs(ROBOT_POSITION.x - ballPoses.get(i).x);
            //linear function where 0 x diff corresponds to weighting of 0.5 and x-diff of 48 corresponds to weighting of 0
            weights[i] = weights[i] +  -0.0104167 * xDifference + 0.5;
        }

        //now we take the distance between the previous ball and the next ball
        for(int i = 1; i < weights.length - 1; i++) {
            double totalXDiff = Math.abs(ballPoses.get(i).x - ballPoses.get(i + 1).x) + Math.abs(ballPoses.get(i).x - ballPoses.get(i - 1).x);
            weights[i] = weights[i] +  -0.00806452 * totalXDiff + 0.580645;
        }

        return weights;
    }

    public ArrayList<Vector2> getBallPoses() {
        return ballPoses;
    }
}

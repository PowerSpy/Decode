package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.LLOffsetForward;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
public class BallDetection {
    private final Limelight3A limelight;

    private final ArrayList<Vector2> ballPoses = new ArrayList<>();

    public static double stalenessThreshMs = 100;
    public static double confidenceThresh = 0; // Todo - Tune

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

        ballPoses.clear();

        LLResult result = limelight.getLatestResult();

        Log.i("BallDetection limelight connected", "" + limelight.isConnected());

        if (result != null) {
            Log.i("BallDetection status", result.isValid() + " " + result.getStaleness());
        }

        if (result != null && result.isValid() && result.getStaleness() < stalenessThreshMs) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            Log.i("BallDetection number of Detections", String.valueOf(detections.size()));

            for (LLResultTypes.DetectorResult detection : detections) {
                if (detection.getConfidence() > confidenceThresh) {

                        double tx = detection.getTargetXDegrees();
                        double ty = detection.getTargetYDegrees();
                        double ta = detection.getTargetArea();

                        if(!Double.isNaN(tx) && !Double.isNaN(ty) && !Double.isNaN(ta)) {

                            Pose2d cameraPos = new Pose2d(ROBOT_POSITION.x + LLOffsetForward * Math.cos(ROBOT_POSITION.heading), ROBOT_POSITION.y + LLOffsetForward * Math.sin(ROBOT_POSITION.heading), ROBOT_POSITION.heading);
                            Log.i("Camera pos is", cameraPos.toString() + "Robot pos is" + ROBOT_POSITION.toString());

                            //lowk in inches
                            double d = 70 - 100 * Math.cbrt(ta - 0.005);
                            Log.i("BallDetection values", tx + " " + ty + " " + d + " " + ta);

                            //funny polar to cartesian conversion
                            double angle = ROBOT_POSITION.heading + Math.toRadians(tx);
                            Vector2 ballPos = new Vector2(cameraPos.x + d * Math.cos(angle), cameraPos.y + d * Math.sin(angle));
                            if (ballPos.x >= 0 && ballPos.y * (Globals.isRed ? 1 : -1) >= 0) {
                                ballPoses.add(ballPos);
                            }

                        } else {
                            Log.i("Ball", "Number was not a number");
                        }

                } else {
                    Log.i("BallDetection", "Confidence too low " + detection.getConfidence());
                }
            }
        } else {
            Log.i("BallDetection", "Result is invalid");
        }

        ballPoses.sort(Comparator.comparingDouble((Vector2 l) -> l.x));
    }

    public double[] getWeights(ArrayList<Vector2> ballPoses) {
        //list of criteria - we want the x difference between robot and ball to be as less as possible
        //we also weight the balls higher if there are balls near it in terms of x
        //weights are clamped between 1 and 0 with 1 being the perfect ball with there being 0 x differnce and two balls directly adjacent to it
        if(ballPoses.isEmpty()) {
            return new double[0];
        }
        double[] weights = new double[ballPoses.size()];
        for (int i = 0; i < weights.length; i++) {
            double xDifference = Math.abs(ROBOT_POSITION.x - ballPoses.get(i).x);
            //linear function where 0 x diff corresponds to weighting of 0.5 and x-diff of 48 corresponds to weighting of 0
            weights[i] = weights[i] +  -0.0104167 * xDifference + 0.5;
        }

        //now we take the distance between the previous ball and the next ball
        for (int i = 1; i < weights.length - 1; i++) {
            double totalXDiff = Math.abs(ballPoses.get(i).x - ballPoses.get(i + 1).x) + Math.abs(ballPoses.get(i).x - ballPoses.get(i - 1).x);
            weights[i] = weights[i] +  -0.00806452 * totalXDiff + 0.580645;
        }

        for (int i = 0; i < weights.length; i++) {
            weights[i] = Utils.minMaxClip(weights[i], 0, 1);
        }
        //punish the first and last balls
        weights[0] = weights[0] - 0.25;
        weights[weights.length - 1] = weights[weights.length - 1] - 0.25;

        return weights;
    }

    public ArrayList<Vector2> getBallPoses() {
        return (ArrayList<Vector2>) ballPoses.clone();
    }

    public Vector2 getBestBall() {return getBestBall(this.getBallPoses());}

    public Vector2 getBestBall(ArrayList<Vector2> ballPoses) {

        double[] w = this.getWeights(ballPoses);

        if (ballPoses.isEmpty()) {
            return null;
        } else {
            double min = -1000000;
            int bestIndex = 0;
            for(int i = 0; i < w.length; i++) {
                if(w[i] > min) {
                    min = w[i];
                    bestIndex = i;
                }
            }
            return ballPoses.get(bestIndex);
        }

    }
}

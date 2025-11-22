package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

public class LLGoalDetector {
    private Robot robot;
    private Limelight3A ll;
    private double tx, ty, ta;
    private boolean connection, tagDetected;

    public static int pollRate = 100;

    public LLGoalDetector(Robot robot){
        this.robot = robot;
        ll = robot.hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(pollRate);
    }

    public void start(){
        ll.pipelineSwitch(Globals.isRed ? 0 : 1);
        ll.start();
    }

    public void stop(){
        ll.stop();
    }

    public void update(){
        if(!ll.isConnected()){
            TelemetryUtil.packet.put("Limelight : Status", "Oops! Something broke :blehhh:");
            connection = false;
        }else{
            TelemetryUtil.packet.put("Limelight : Status", "Connected");
            connection = true;
        }

        if(connection){
            LLResult result = ll.getLatestResult();
            if(result != null && result.isValid()){
                tagDetected = true;
                tx = result.getTx();
                ty = result.getTy();
                ta = result.getTa();
            }else{
                tagDetected = false;
            }
        }
    }

    public double getTx(){
        return tx;
    }

    public double getTy(){
        return ty;
    }

    public double getTa(){
        return ta;
    }

    public boolean isTagDetected(){
        return tagDetected;
    }
}

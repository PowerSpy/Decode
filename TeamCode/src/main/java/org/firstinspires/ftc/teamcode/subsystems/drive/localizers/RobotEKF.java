package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

/*
The sensor inputs into the kalman filter are the following:
 Pinpoint - x,y, theta
 3 wheel odo - x,y, theta
 Apriltag - x,y, theta

 Note:
 Everything is in inches and radians! (Rishi I commented Units yo!)
 */
public class RobotEKF {

    //this is our robot state [x, y, theta]
    private double x;
    private double y;
    private double theta;

    //P is covariance matrix Q is the noise matrix
    private final double[][] P = new double[3][3];
    private final double[][] Q = new double[3][3];

    //tune the below values these are the values that represent how much variance each of them have.
    public static double visionRX = 0.04, visionRY = 0.04, visionRT = 0.02;
    public static double pinpointRX = 0.01, pinpointRY = 0.01, pinpointRT = 0.05;
    public static double odoRX = 0.1, odoRY = 0.1, odoRT = 0.06;



    public RobotEKF(Pose2d startPose, double qX, double qY, double qTheta) {
        this.x     = startPose.x;
        this.y     = startPose.y;
        this.theta = startPose.heading;

        setDiagonal(P, new double[]{0.01, 0.01, 0.01});

        setDiagonal(Q, new double[]{qX, qY, qTheta});
    }

    //starting with noise
    public RobotEKF(Pose2d startPose) {
        this(startPose, 0.01, 0.01, 0.01);
    }

    //prediction phase of kalman filter
    public void predict(double vxField, double vyField, double omega, double dt) {
        x     += vxField * dt;
        y     += vyField * dt;
        theta  = AngleUnit.normalizeRadians(theta + omega * dt);

        //Since there is no correlation between the x,y, and theta and each of them act indpendently F · P · Fᵀ  +  Q becomes P + Q as F is the identity matrix bc of no correlations
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P[i][j] += Q[i][j] * dt;
            }
        }
    }

    //feeding the sensors into the covariance matrix
    public void updateSensor(double zx, double zy, double zt,
                               double rX, double rY, double rT) {
        double[] innov = {
                zx - x,
                zy - y,
                AngleUnit.normalizeRadians(zt - theta)
        };
        double[] R = {rX, rY, rT};
        fullUpdate3(innov, R);
    }

    public void updatePinpoint(double zx, double zy, double zt) {
        updateSensor(zx, zy, zt, pinpointRX, pinpointRY, pinpointRT);
    }

    public void updateOdometry(double zx, double zy, double zt) {
        updateSensor(zx, zy, zt, odoRX, odoRY, odoRT);
    }

    public void updateAprilTag(double zx, double zy, double zt) {
        updateSensor(zx, zy, zt, visionRX, visionRY, visionRT);
    }

    public boolean isAprilTagOutlier(double zx, double zy, double maxPosDelta) {
        double dx = zx - x;
        double dy = zy - y;
        return (dx * dx + dy * dy) > (maxPosDelta * maxPosDelta);
    }

    public void updateSensorWithInnovation(double innovX, double innovY, double innovTheta) {
        double[] innov = {
                innovX,
                innovY,
                AngleUnit.normalizeRadians(innovTheta)
        };
        fullUpdate3(innov, new double[]{visionRX, visionRY, visionRT});
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, theta);
    }

    public double getX()     { return x; }
    public double getY()     { return y; }
    public double getTheta() { return theta; }

    public double[][] getCovarianceCopy() {
        double[][] copy = new double[3][3];
        for (int i = 0; i < 3; i++)
            System.arraycopy(P[i], 0, copy[i], 0, 3);
        return copy;
    }

    public void resetPose(Pose2d pose) {
        this.x     = pose.x;
        this.y     = pose.y;
        this.theta = pose.heading;
        setDiagonal(P, new double[]{0.01, 0.01, 0.01});
    }

    private void fullUpdate3(double[] innov, double[] R) {
        // S = P + diag(R)  (3x3)
        double[][] S = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                S[i][j] = P[i][j];
            }
            S[i][i] += R[i];
        }

        // Invert S (3x3) via Gauss-Jordan
        double[][] Sinv = invert3x3(S);
        if (Sinv == null) return;

        double[][] K = mult3x3(P, Sinv);

        // State update
        x     += K[0][0] * innov[0] + K[0][1] * innov[1] + K[0][2] * innov[2];
        y     += K[1][0] * innov[0] + K[1][1] * innov[1] + K[1][2] * innov[2];
        theta  = AngleUnit.normalizeRadians(
                theta + K[2][0] * innov[0] + K[2][1] * innov[1] + K[2][2] * innov[2]);

        //Joseph covariance update: P = (I-K)*P*(I-K)^T + K*R*K^T
        josephUpdate3(K, R);
    }

    private void josephUpdate3(double[][] K, double[] R) {
        // A = I - K
        double[][] A = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                A[i][j] = (i == j ? 1.0 : 0.0) - K[i][j];
            }
        }

        // A * P * A^T
        double[][] AP   = mult3x3(A, P);
        double[][] APAt = mult3x3(AP, transpose3x3(A));

        // K * diag(R) * K^T
        double[][] KR = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                KR[i][j] = K[i][0] * R[0] * K[j][0]
                        + K[i][1] * R[1] * K[j][1]
                        + K[i][2] * R[2] * K[j][2];

        // P = APAt + KRKt
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                P[i][j] = APAt[i][j] + KR[i][j];
    }

    //bunch of matrix functions specifically for the kalman filter 3x3 matrices
    private static void setDiagonal(double[][] M, double[] vals) {
        int n = vals.length;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                M[i][j] = (i == j) ? vals[i] : 0.0;
    }

    private static void add3x3(double[][] A, double[][] B) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                A[i][j] += B[i][j];
    }

    private static double[][] mult3x3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    private static double[][] transpose3x3(double[][] A) {
        double[][] T = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                T[j][i] = A[i][j];
        return T;
    }

    private static double[][] invert3x3(double[][] M) {
        double det =
                M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                        - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                        + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

        if (Math.abs(det) < 1e-9) return null;

        double invDet = 1.0 / det;
        double[][] inv = new double[3][3];

        inv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * invDet;
        inv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * invDet;
        inv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * invDet;
        inv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * invDet;
        inv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * invDet;
        inv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * invDet;
        inv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * invDet;
        inv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * invDet;
        inv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * invDet;

        return inv;
    }
}
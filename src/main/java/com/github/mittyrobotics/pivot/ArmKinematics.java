package com.github.mittyrobotics.pivot;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import org.ejml.simple.SimpleMatrix;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class ArmKinematics {
    private static Angle pitch = new Angle(0);

    static double incrementSpeed = 0.0005;

    private static double tuningHeight = 5/39.37;
    private static double radius = 0;
    private static double tuningDistance = 0;

    public static void setArmKinematics(double distance, double height) {
        radius = Math.sqrt(distance * distance + height * height);
        pitch = new Angle(Math.atan2(height, distance));
    }

    public static void setArmKinematics(Angle theta, double r) {
        radius = r;
        pitch = theta;
    }

    public static Angle getPivotDesired() {
        return pitch;
    }

    public static double getTelescopeDesired() {
        return radius;
    }

    public static void incrementDistance(boolean extend) {
        System.out.println("TUNING DISTANCE: " + tuningDistance);
        if (extend) {
            setArmKinematics(tuningDistance+=incrementSpeed, tuningHeight);
        } else {
            setArmKinematics(tuningDistance-=incrementSpeed, tuningHeight);
        }
    }

    public SimpleMatrix getCameraRotationMatrix(double alpha, double beta, double gamma) {
        SimpleMatrix Rz = new SimpleMatrix(new double[][]
                {{Math.cos(alpha), -Math.sin(alpha), 0},
                 {Math.sin(alpha), Math.cos(alpha), 0},
                 {0, 0, 1}});
        SimpleMatrix Ry = new SimpleMatrix(new double[][]
                {{Math.cos(beta), 0, Math.sin(beta)},
                 {0, 1, 0},
                 {-Math.sin(beta), 0, Math.cos(beta)}});
        SimpleMatrix Rx = new SimpleMatrix(new double[][]
                {{1, 0, 0},
                 {0, Math.cos(gamma), -Math.sin(gamma)},
                 {0, Math.sin(gamma), Math.cos(gamma)}});
        return Rz.mult(Ry).mult(Rx);
    }

    public SimpleMatrix getCameraRotationMatrix() {
        return getCameraRotationMatrix(0, Math.PI/2 - PivotSubsystem.getInstance().getPositionRadians(), 0);
    }

    public static void incrementHeight(boolean up) {
        System.out.println("TUNING HEIGHT: " + tuningHeight);
        if (up) {
            setArmKinematics(tuningDistance, tuningHeight+=incrementSpeed);
        } else {
            setArmKinematics(tuningDistance, tuningHeight-=incrementSpeed);
        }
    }

    public static void incrementLinear(double joystickY, double joystickX) {
        setArmKinematics(0.75 * Math.pow(joystickX,4), Math.pow(joystickY, 4)*0.75);
    }

    public Pose getCameraPose() {
        return getCameraPose(PivotSubsystem.getInstance().getPositionRadians());
    }

    public Pose getCameraPose(double phi) {
        return new Pose(
                new Point(
                        7.5625 * Math.sin(phi) / 39.37,
                        (25 + 1.87623032 + 7.5625 * Math.cos(phi)) / 39.37),
                new Angle(-phi));
    }

    public Vector getVectorToGamePiece(double dist, double theta, double phi) {
        return Vector.add(new Vector(dist * Math.sin(phi) * Math.cos(theta), dist * Math.sin(phi) * Math.sin(theta)), new Vector(getCameraPose().getPosition()));
    }

    public Vector getVectorToGamePiece(boolean isCube, int index) throws JSONException {
        JSONObject object = LoggerInterface.getInstance().getGamePiece()
                .getJSONArray(isCube ? "cubes" : "cones").getJSONObject(index);
        return getVectorToGamePiece(
                object.getDouble("distance") / 100.,
                object.getDouble("anglex") * Math.PI / 180,
                (90 - object.getDouble("angley")) * Math.PI / 180.);
    }
}

package com.github.mittyrobotics;

import org.json.JSONException;
import org.json.JSONObject;

public class TestJsonLib {
    static LoggerInterface loggerInterface;
    public static void main(String[] args) throws JSONException {
//        JSONObject obj = new JSONObject("{cones: [5, 6, 7, 8]}");
//        System.out.println(obj.getJSONArray("cones"));
        loggerInterface = new LoggerInterface();
        System.out.println(loggerInterface.getGamePiece().getJSONArray("cones"));
    }
}

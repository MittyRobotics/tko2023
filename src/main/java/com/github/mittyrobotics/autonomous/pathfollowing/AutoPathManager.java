package com.github.mittyrobotics.autonomous.pathfollowing;

public class AutoPathManager {
    private static SwervePath currentPath;

    public static SwervePath getCurrentPath() {
        return currentPath;
    }

    public static void setCurrentPath(SwervePath currentPath) {
        AutoPathManager.currentPath = currentPath;
    }
}

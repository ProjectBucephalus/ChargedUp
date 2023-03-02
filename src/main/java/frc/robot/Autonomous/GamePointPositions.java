package frc.robot;

public class GamePointPositions {
    String name;
    double x;
    double y;
    double z;
    GamePointPositions inst;

    private GamePointPositions(String name,double x,double y,double z){
        this.x = x;
        this.y = y;
        this.z = z;
        this.name = name;
    }

    public static GamePointPositions gamePointPositions(String string, double x, double y, double z) {
        return gamePointPositions(string, x, y, z);
    }

}
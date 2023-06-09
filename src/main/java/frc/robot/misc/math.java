package frc.robot.misc;

public final class math {
    private math(){}

    public static int mod(int x, int m){
        return ((x % m) + m) % m;
    }
}

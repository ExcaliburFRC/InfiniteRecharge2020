package frc.robot.subsystems;

public class BooleanAverager{
    private int location, size;
    private boolean[] bucket;
    public BooleanAverager(int bucketSize){
        this.size = bucketSize;
        this.location = 0;
        bucket = new boolean[this.size];

        for (int i = 0 ; i < size; i++){
            bucket[i] = false;
        }
    }

    public void update(boolean val){
        bucket[location] = val;
        location = (location + 1) % size;
    }

    public boolean getAverage(){
        int trueNum = 0;
        for (boolean i : bucket){
            if (i) trueNum++;
        }
        return trueNum > size/2;
    }
}
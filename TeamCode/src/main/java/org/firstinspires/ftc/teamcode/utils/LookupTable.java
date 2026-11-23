package org.firstinspires.ftc.teamcode.utils;

// a cheat sheet of distance -> value that worked
// figures out the in between distances by itself
public class LookupTable {

    private final double[] distances;
    private final double[] values;

    // distances have to go smallest to biggest
    public LookupTable(double[] distances, double[] values) {
        // crash right away if the table is set up wrong, not mid match
        if (distances.length != values.length) {
            throw new IllegalArgumentException("distances and values need the same number of rows");
        }
        if (distances.length < 2) {
            throw new IllegalArgumentException("need at least 2 rows");
        }
        for (int i = 1; i < distances.length; i++) {
            if (distances[i] <= distances[i - 1]) {
                throw new IllegalArgumentException("distances have to go smallest to biggest");
            }
        }

        // copies so nobody can change the table after
        this.distances = distances.clone();
        this.values = values.clone();
    }

    // true if this distance is inside the table
    // outside it we dont trust the shot
    public boolean inRange(double distance) {
        return distance >= distances[0] && distance <= distances[distances.length - 1];
    }

    // gets the value for this distance
    public double get(double distance) {
        // closer than the first row, use the first row
        if (distance <= distances[0]) return values[0];

        // farther than the last row, use the last row
        int last = distances.length - 1;
        if (distance >= distances[last]) return values[last];

        // find the first row thats farther than us
        int i = 1;
        while (distances[i] < distance) i++;

        // how far we are between the two rows, 0 to 1
        double t = (distance - distances[i - 1]) / (distances[i] - distances[i - 1]);

        // go that far between the two values
        return values[i - 1] + t * (values[i] - values[i - 1]);
    }
}

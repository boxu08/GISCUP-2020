package COMSETsystem;

/**
 * Location on a road represented by the travel time from the start intersection of the road.
 */
public class StaticTravelTimeLocationOnRoad {

    public Road road;
    public long travelTimeFromStartIntersection;

    public StaticTravelTimeLocationOnRoad(Road road, long travelTimeFromStartIntersection) {
        this.road = road;
        this.travelTimeFromStartIntersection = travelTimeFromStartIntersection;
    }

    public StaticTravelTimeLocationOnRoad(DistanceLocationOnLink distanceLocationOnLink) {
        this.road = distanceLocationOnLink.link.road;
        Link link = distanceLocationOnLink.link;
        long travelTimeFromStartVertex = Math.round(distanceLocationOnLink.distanceFromStartVertex / link.length * link.travelTime);
        this.travelTimeFromStartIntersection = link.beginTime + travelTimeFromStartVertex;
    }

    /**
     * Get a lat,lon representation of the location
     *
     * @return double[] a double array {lat, lon}
     */
    public double[] toLatLon() {
        double latLon[] = new double[2];
        int i;
        for (i = 0; i < road.links.size() && road.links.get(i).beginTime <= this.travelTimeFromStartIntersection; i++);
        i--;
        // interpolate
        Link link = road.links.get(i);
        long travelTimeFromStartVertex = this.travelTimeFromStartIntersection - link.beginTime;
        if (link.travelTime == 0) {
            latLon[0] = (link.from.latitude + link.to.latitude) / 2;
            latLon[1] = (link.from.longitude + link.to.longitude) / 2;
        } else {
            latLon[0] = link.from.latitude + (link.to.latitude - link.from.latitude) * (((double)travelTimeFromStartVertex) / link.travelTime);
            latLon[1] = link.from.longitude + (link.to.longitude - link.from.longitude) * (((double)travelTimeFromStartVertex) / link.travelTime);
        }

        return latLon;
    }

    public String toString() {
        double latLon[] = toLatLon();
        return "(" + road + "," + this.travelTimeFromStartIntersection + ",(" + latLon[0] + "," + latLon[1] + "))";
    }
}


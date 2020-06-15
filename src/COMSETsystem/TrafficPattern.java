package COMSETsystem;

import COMSETsystem.DistanceLocationOnLink;
import COMSETsystem.Link;
import COMSETsystem.Road;
import COMSETsystem.Vertex;

import java.util.ArrayList;

public class TrafficPattern {
    public long epoch;
    public long step;

    public ArrayList<TrafficPatternItem> trafficPattern;
    public TrafficPattern(long epoch, long step) {
        this.epoch = epoch;
        this.step = step;
        this.trafficPattern = new ArrayList<TrafficPatternItem>();
    }

    public void addTrafficPatternItem(long epochBeginTime, double speedFactor) {
        TrafficPatternItem trafficPatternItem = new TrafficPatternItem(epochBeginTime, speedFactor);
        trafficPattern.add(trafficPatternItem);
    }

    // get the speed factor of a given time
    public double getSpeedFactor(double time) {
        if (time < trafficPattern.get(0).epochBeginTime) {
            return trafficPattern.get(0).speed_factor;
        } else if (time >= trafficPattern.get(trafficPattern.size()-1).epochBeginTime) {
            return trafficPattern.get(trafficPattern.size()-1).speed_factor;
        } else {
            // compute the index of time
            int patternIndex = (int)((time - trafficPattern.get(0).epochBeginTime) / step);
            assert(time >= trafficPattern.get(patternIndex).epochBeginTime && time <= trafficPattern.get(patternIndex).epochBeginTime + step);
            return trafficPattern.get(patternIndex).speed_factor;
        }
    }

    public void printOut() {
        for (TrafficPatternItem trafficPatternItem : trafficPattern) {
            System.out.println(trafficPatternItem.epochBeginTime + "," + trafficPatternItem.speed_factor);
        }
    }

    class TrafficPatternItem {
        public long epochBeginTime;
        public double speed_factor;
        public TrafficPatternItem(long epochBeginTime, double speed_factor) {
            this.epochBeginTime = epochBeginTime;
            this.speed_factor = speed_factor;
        }
        public String toString() {
            return String.valueOf(this.epochBeginTime)+","+String.valueOf(this.speed_factor);
        }
    }

    // compute the dynamic travel time to travel a certain distance of a link starting at a certain time
    public double dynamicForwardTravelTime(double time, double unadjustedSpeed, double distance) {
        double totalDistance = 0.0;
        double totalTime = 0.0;
        double currentTime = time;

        double speedFactor;
        double adjustedSpeed;

        if (time >= this.trafficPattern.get(this.trafficPattern.size()-1).epochBeginTime) {
            speedFactor = this.trafficPattern.get(this.trafficPattern.size()-1).speed_factor;
            adjustedSpeed = unadjustedSpeed * speedFactor;
            return distance / adjustedSpeed;
        }
        while (true) {
            double stepTime = -1.0;
            // travel for one step
            if (currentTime >= this.trafficPattern.get(this.trafficPattern.size()-1).epochBeginTime) {
                speedFactor = this.trafficPattern.get(this.trafficPattern.size()-1).speed_factor;
                adjustedSpeed = unadjustedSpeed * speedFactor;
                stepTime = (distance - totalDistance) / adjustedSpeed;
                totalTime += stepTime;
                break;
            } else {
                if (currentTime < this.trafficPattern.get(0).epochBeginTime) {
                    stepTime = this.trafficPattern.get(0).epochBeginTime - currentTime;
                    speedFactor = this.trafficPattern.get(0).speed_factor;
                } else {
                    int patternIndex = (int) ((currentTime - trafficPattern.get(0).epochBeginTime) / step);
                    stepTime = trafficPattern.get(patternIndex).epochBeginTime + step - currentTime;
                    speedFactor = this.trafficPattern.get(patternIndex).speed_factor;
                }
                adjustedSpeed = unadjustedSpeed * speedFactor;
                double stepDistance = adjustedSpeed * stepTime;
                if (totalDistance + stepDistance < distance) {
                    // finish a full step
                    totalDistance += stepDistance;
                    totalTime += stepTime;
                    currentTime += stepTime;
                } else {
                    // finish a partial step
                    double remainingDistance = distance - totalDistance;
                    double remainingTime = remainingDistance / adjustedSpeed;
                    totalTime += remainingTime;
                    break;
                }
            }
        }
        return totalTime;
    }

    // compute the dynamic travel time to travel a certain distance of a link ending at a certain time
    public double dynamicBackwardTravelTime(double time, double unadjustedSpeed, double distance) {
        double totalDistance = 0.0;
        double totalTime = 0.0;
        double currentTime = time;
        while (true) {
            // travel for one step
            double speedFactor = this.getSpeedFactor(currentTime);
            double adjustedSpeed = unadjustedSpeed * speedFactor;
            double stepTime = -1.0;
            if (currentTime < this.trafficPattern.get(0).epochBeginTime) {
                stepTime = (this.trafficPattern.get(0).epochBeginTime - currentTime);
            } else if (currentTime >= this.trafficPattern.get(this.trafficPattern.size()-1).epochBeginTime) {
                stepTime = currentTime - this.trafficPattern.get(this.trafficPattern.size()-1).epochBeginTime;
            } else {
                stepTime = this.step;
            }
            double stepDistance = adjustedSpeed * stepTime;
            if (totalDistance + stepDistance < distance) {
                // finish a full step
                totalDistance += stepDistance;
                totalTime += stepTime;
                currentTime += stepTime;
            } else {
                // finish a partial step
                double remainingDistance = distance - totalDistance;
                double remainingTime = remainingDistance / adjustedSpeed;
                totalTime += remainingTime;
                break;
            }
        }
        return totalTime;
    }

    // how long
    public double linkForwardTravelTime(double time, Link link, double distance) {
        double travelTime = dynamicForwardTravelTime(time, link.speed, distance);
        return travelTime;
    }

    // dynamic travel time from a location on a road to the end of the road
    public double roadTravelTimeToEndIntersection(double time, Road road, Link link, double distanceFromStartVertex) {
        double totalTravelTime = 0.0;
        double currentTime = time;
        Link currentLink = link;
        double travelTime = linkForwardTravelTime(currentTime, link, link.length - distanceFromStartVertex);
        totalTravelTime += travelTime;
        currentTime += travelTime;
        int link_order = road.links.indexOf(currentLink);
        for (int i = link_order + 1; i < road.links.size(); i++) {
            currentLink = road.links.get(i);
            travelTime = linkForwardTravelTime(currentTime, currentLink, currentLink.length);
            totalTravelTime += travelTime;
            currentTime += travelTime;
        }
        return totalTravelTime;
    }

    // dynamic travel time from a location on a road to the end of the road
    public double roadTravelTimeToEndIntersection(double time, Link link, double distanceFromStartVertex) {
        return roadTravelTimeToEndIntersection(time, link.road, link, distanceFromStartVertex);
    }

    // dynamic travel time from a location on a road to the end of the road
    public double roadTravelTimeToEndIntersection(double time, DistanceLocationOnLink loc) {
        return roadTravelTimeToEndIntersection(time, loc.link, loc.distanceFromStartVertex);
    }

    // dynamic t time from the start of a road to a location on the road
    // TODO: to be tested!!
    public double roadTravelTimeFromStartIntersection(double time, DistanceLocationOnLink loc) {
        double totalTravelTime = 0.0;
        double currentTime = time;

        Link targetLink = loc.link;
        for (Link currentLink : loc.link.road.links) {
            if (currentLink.id == targetLink.id) {
                break;
            }
            double travelTime = linkForwardTravelTime(currentTime, currentLink, currentLink.length);
            totalTravelTime += travelTime;
            currentTime += travelTime;
        }
        totalTravelTime = linkForwardTravelTime(currentTime, targetLink, loc.distanceFromStartVertex);
        return totalTravelTime;
    }


    public static void roadTravelTimeTest_case_1() {
        // configure a traffic pattern
        TrafficPattern trafficPattern = new TrafficPattern(6L, 6L);
        trafficPattern.addTrafficPatternItem(0L, 0.5);
        trafficPattern.addTrafficPatternItem(6L, 1);
        trafficPattern.addTrafficPatternItem(12L, 2);

        // configure links
        Vertex vertex1 = new Vertex(0, 0, 0, 0, 0);
        Vertex vertex2 = new Vertex(0, 0, 10, 0, 1);
        Vertex vertex3 = new Vertex(0, 0, 22, 0, 2);
        Vertex vertex4 = new Vertex(0, 0, 28, 0, 3);
        Link link1 = new Link(vertex1, vertex2, 10, 4);
        Link link2 = new Link(vertex2, vertex3, 12, 2);
        Link link3 = new Link(vertex3, vertex4, 6, 1);

        ArrayList<Link> links = new ArrayList<Link>();
        links.add(link1);
        links.add(link2);
        links.add(link3);
        // configure a road
        Road road = new Road();
        road.links = links;

        double travelTime = trafficPattern.roadTravelTimeToEndIntersection(3, road, link1, 2);
        System.out.println("calculated travel time = " + travelTime + ", correct travel time = " + 12.25);
    }

    public static void roadTravelTimeTest_case_2() {
        // configure a traffic pattern
        TrafficPattern trafficPattern = new TrafficPattern(4L, 4L);
        trafficPattern.addTrafficPatternItem(0L, 2);
        trafficPattern.addTrafficPatternItem(4L, 1);
        trafficPattern.addTrafficPatternItem(8L, 3);
        trafficPattern.addTrafficPatternItem(12L, 4);

        // configure links
        Vertex vertex1 = new Vertex(0, 0, 0, 0, 0);
        Vertex vertex2 = new Vertex(0, 0, 10, 0, 1);
        Vertex vertex3 = new Vertex(0, 0, 22, 0, 2);
        Vertex vertex4 = new Vertex(0, 0, 28, 0, 3);
        Link link1 = new Link(vertex1, vertex2, 10, 4);
        Link link2 = new Link(vertex2, vertex3, 12, 2);
        Link link3 = new Link(vertex3, vertex4, 6, 1);

        ArrayList<Link> links = new ArrayList<Link>();
        links.add(link1);
        links.add(link2);
        links.add(link3);
        // configure a road
        Road road = new Road();
        road.links = links;

        //double travelTime = trafficPattern.roadForwardTravelTime(3, road, link1, 2);
        //System.out.println("calculated travel time = " + travelTime + ", correct travel time = " + 7.67);
        double travelTime = trafficPattern.roadTravelTimeToEndIntersection(5, road, link2, 3);
        System.out.println("calculated travel time = " + travelTime + ", correct travel time = " + 5.5);
    }

    // test code
    public static void main(String[] args) {
        //roadTravelTimeTest_case_1();
        roadTravelTimeTest_case_2();
    }

    public static void travelTimeTest() {
        TrafficPattern trafficPattern = new TrafficPattern(10L, 3L);
        trafficPattern.addTrafficPatternItem(100L, 0.5);
        trafficPattern.addTrafficPatternItem(103L, 1);
        trafficPattern.addTrafficPatternItem(106L, 2);
        double travelTime1 = trafficPattern.dynamicForwardTravelTime(102L, 2, 13);
        // true travel time = 5.5 because 1*0.5*2 + 3*1*2 + 1.5*2*2 = 13
        System.out.println("calculated travel time = " + travelTime1 + ", " + "correct travel time = " + 5.5);
        double travelTime2 = trafficPattern.dynamicForwardTravelTime(99L, 2, 28);
        // true travel time = 11.5 because 1*0.5*2 + 3*0.5*2 + 3*1*2 + 3*2*2 + 1.5*2*2 = 28
        System.out.println("calculated travel time = " + travelTime2 + ", " + "correct travel time = " + 11.5);
        double travelTime3 = trafficPattern.dynamicForwardTravelTime(120, 2, 38);
        // travel travel time = 9.5 because 9.5*2*2 = 38
        System.out.println("calculated travel time = " + travelTime3 + ", " + "correct travel time = " + 9.5);
    }


}


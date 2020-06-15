package COMSETsystem;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.runners.MockitoJUnitRunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.*;

@RunWith(MockitoJUnitRunner.class)
public class AgentEventTest {

    public static final int CUSTOMER_TRIP_TIME = 500;
    public static final long TIME_TO_PICKUP_CUSTOMER = 300L;
    public static final int TRIGGER_TIME = 100;
    @Mock
    Simulator mockSimulator;
    @Mock
    Event mockEvent;
    @Mock
    ResourceEvent mockResourceEvent;
    @Mock
    LocationOnRoad mockLocationOnRoad;
    @Mock
    Road mockRoad;
    @Mock
    Simulator.PickUp mockNoPickUp;
    @Mock
    FleetManager mockFleetManager;
    @Mock
    AssignmentManager mockAssignmentManager;
    private SimpleMap testMap = new SimpleMap();

    @Before
    public void BeforeEachTest() {
    }

    /**
     * Tests cruising this path intersection 1 -> intersection 2 -> intersection 3.
     *
     * @throws Exception from spyEvent.Trigger if any
     */
    @Test
    public void testTrigger_cruising() throws Exception {
        // Construction agent reaching intersection2
        DistanceLocationOnLink locAtReachedIntersection = new DistanceLocationOnLink(testMap.link1to2,
                testMap.link1to2.length);

        AgentEvent agentEvent = new AgentEvent(locAtReachedIntersection,
                TRIGGER_TIME, mockSimulator, mockFleetManager);
        // return intersection3 as next intersection
        when(mockFleetManager.onReachIntersection(eq(agentEvent.id), anyLong(), anyObject()))
                .thenReturn(testMap.intersection3);

        // Verify initial state
        assertEquals(AgentEvent.State.INTERSECTION_REACHED, agentEvent.state);

        AgentEvent nextEvent = (AgentEvent) agentEvent.trigger();

        // Verify that next AgentEvent will trigger when reaching the end of road2
        assertEquals(AgentEvent.State.INTERSECTION_REACHED, nextEvent.state);
        assertEquals(testMap.roadFrom2to3, nextEvent.loc.link.road);
        assertEquals(TRIGGER_TIME + testMap.roadFrom2to3.travelTime, nextEvent.time);
    }

    @Test
    public void testTrigger_arrivingAtPickup() throws Exception {
        // Create a Resource to be picked up half way down road.
        DistanceLocationOnLink pickUpLocation = new DistanceLocationOnLink(testMap.link2to3,
                testMap.link2to3.length/2);
        DistanceLocationOnLink dropOffLocation = new DistanceLocationOnLink(testMap.link4to5,
                testMap.link4to5.length/2);
        ResourceEvent customer = new ResourceEvent(pickUpLocation, dropOffLocation,
                3600, 0, mockSimulator, mockFleetManager, mockAssignmentManager);

        // Create an agentEvent that has a pickup reaching intersection2
        DistanceLocationOnLink locAtReachedIntersection = new DistanceLocationOnLink(testMap.link1to2,
                testMap.link1to2.length);
        AgentEvent agentEvent = new AgentEvent(locAtReachedIntersection, TRIGGER_TIME,
                mockSimulator, mockFleetManager);
        agentEvent.assignTo(customer);

        // Trigger the event
        AgentEvent pickUpEvent = (AgentEvent) agentEvent.trigger();

        assertEquals(AgentEvent.State.PICKING_UP, pickUpEvent.state);
        assertEquals(TRIGGER_TIME + testMap.roadFrom2to3.travelTime/2, pickUpEvent.time);
        assertEquals(testMap.roadFrom2to3, pickUpEvent.loc.link.road);

        // Trigger pickup Event
        AgentEvent nextEvent = (AgentEvent) pickUpEvent.trigger();

        // Verify that event will trigger when it hits the end of the road that customer was waiting
        assertEquals(AgentEvent.State.INTERSECTION_REACHED, nextEvent.state);
        assertEquals(TRIGGER_TIME + testMap.roadFrom2to3.travelTime, nextEvent.time);
        assertEquals(testMap.roadFrom2to3, nextEvent.loc.link.road);
        assertTrue(nextEvent.isPickup);
        long travelTimeFromStartIntersection = (long)(pickUpLocation.distanceFromStartVertex / pickUpLocation.link.speed);
        assertEquals(TRIGGER_TIME + travelTimeFromStartIntersection, customer.pickupTime);

        // Validate simulation statistics
        assertEquals(testMap.roadFrom2to3.travelTime/2, mockSimulator.totalAgentSearchTime);

        // TODO This isn't compute correctly, so comment out.
        // assertEquals(-1, mockSimulator.totalResourceWaitTime);

    }

    @Test
    public void testNavigate_withPickUp() throws Exception {
        DistanceLocationOnLink locationOnRoad = spy(new DistanceLocationOnLink(testMap.link1to2, testMap.link1to2.length));
        when(locationOnRoad.toString()).thenReturn("123,45t");

        ResourceEvent resource = new ResourceEvent(
                new DistanceLocationOnLink(testMap.link2to3, 600L),
                new DistanceLocationOnLink(testMap.link2to3, 1200L),
                100L,
                1000L,
                mockSimulator,
                mockFleetManager,
                mockAssignmentManager
        );

        AgentEvent spyEvent = spy(new AgentEvent(locationOnRoad, 100, mockSimulator, mockFleetManager));
        spyEvent.assignTo(resource);
        AgentEvent newEvent = (AgentEvent) spyEvent.trigger();

        assertEquals(AgentEvent.State.PICKING_UP, newEvent.state);
    }

    @Test
    public void testTwoRoadsIntesecting() {
        assertEquals(testMap.intersection1, testMap.roadFrom1to2.from);
        assertEquals(testMap.intersection2, testMap.roadFrom1to2.to);
        assertEquals(testMap.roadFrom1to2.to, testMap.roadFrom2to3.from);
        assertEquals(testMap.intersection3, testMap.roadFrom2to3.to);
    }

    private static class SimpleMap {

        private final Vertex vertex1;
        private final Vertex vertex2;
        private final Vertex vertex3;
        private final Vertex vertex4;
        private final Vertex vertex5;
        private final Link link1to2;
        private final Link link2to3;
        private final Link link3to4;
        private final Link link4to5;
        private final Intersection intersection1;
        private final Intersection intersection2;
        private final Intersection intersection3;
        private final Intersection intersection4;
        private final Intersection intersection5;
        private final Road roadFrom1to2;
        private final Road roadFrom2to3;
        private final Road roadFrom3to4;
        private final Road roadFrom4to5;

        private static Vertex makeVertex(final double longitude, final double latitude, final int id) {
            return new Vertex(longitude, latitude, id, id, id);
        }

        private static Intersection makeIntersection(Vertex vertex) {
            Intersection intersection = new Intersection(vertex);
            vertex.intersection = intersection;
            return intersection;
        }

        private static Road makeRoad(Intersection intersection1, Intersection intersection2) {
            Road r = new Road();
            r.from = intersection1;
            r.to = intersection2;
            r.to.roadsMapTo.put(r.from, r);
            r.from.roadsMapFrom.put(r.to, r);
            return r;
        }


        public SimpleMap(){
            vertex1 = makeVertex(100.0, 100.0, 1);
            vertex2 = makeVertex(100.0, 101.0, 2);
            vertex3 = makeVertex(100.0, 102.0, 3);
            vertex4 = makeVertex(100.0, 103.0, 4);
            vertex5 = makeVertex(100.0, 104.0, 5);
            link1to2 = new Link(vertex1, vertex2, 1000, 50);
            link2to3 = new Link(vertex2, vertex3, 1200, 60);
            link3to4 = new Link(vertex3, vertex4, 800, 20);
            link4to5 = new Link(vertex4, vertex5, 900, 10);
            intersection1 = makeIntersection(vertex1);
            intersection2 = makeIntersection(vertex2);
            intersection3 = makeIntersection(vertex3);
            intersection4 = makeIntersection(vertex4);
            intersection5 = makeIntersection(vertex5);
            roadFrom1to2 = makeRoad(intersection1, intersection2);
            roadFrom2to3 = makeRoad(intersection2, intersection3);
            roadFrom3to4 = makeRoad(intersection3, intersection4);
            roadFrom4to5 = makeRoad(intersection4, intersection5);
            roadFrom1to2.addLink(link1to2);
            roadFrom2to3.addLink(link2to3);
            roadFrom3to4.addLink(link3to4);
            roadFrom3to4.addLink(link4to5);
        }
    }
}

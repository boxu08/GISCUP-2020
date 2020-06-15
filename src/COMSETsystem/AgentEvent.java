package COMSETsystem;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * @author TijanaKlimovic
 * <p>
 * The AgentEvent class represents a moment an agent is going to perform an
 * action in the simmulation, such as becoming empty and picking up a
 * resource, or driving to some other Intersection.
 * <p>
 * An AgentEvent is triggered in either of the following cases:
 * <p>
 * 1. The agent reaches an intersection.
 * 2. The agent drops off a resource.
 * <p>
 * In the case that the agent reaches an intersection, the AgentEvent invokes agent.nextIntersection()
 * to let the agent determine which of the neighboring intersections to go to. The AgentEvent is triggered
 * again when the agent reaches the next intersection, and so on. This is how the agent's search route
 * is executed. The searching ends when the agent is assigned to a resource, in which case the AgentEvent
 * is set to be triggered at the time when the agent drops off the resource to its destination.
 * <p>
 * In the case that the agent drops off a resource, the AgentEvent checks if there are waiting resources. If so,
 * the AgentEvent assigns the agent to the closest waiting resource if the travel time from the agent's current location
 * to the resource is smaller than the resource's remaining life time. Otherwise the AgentEvent moves the agent to
 * the end intersection of the current road.
 */
public class AgentEvent extends Event {

	enum State {
		INTERSECTION_REACHED,
		PICKING_UP,
		DROPPING_OFF
	}

	// The location at which the event is triggered.
	DistanceLocationOnLink loc;

	ResourceEvent assignedResource;

	boolean isPickup = false;

	State state = State.INTERSECTION_REACHED;

	/*
	 * The time at which the agent started to search for a resource. This is also the
	 * time at which the agent drops off a resource.
	 */
	long startSearchTime;

	/**
	 * Constructor for class AgentEvent.
	 *
	 * @param loc this agent's location when it becomes empty.
	 */
	public AgentEvent(DistanceLocationOnLink loc, long startedSearch, Simulator simulator, FleetManager fleetManager) {
		super(startedSearch + (long) simulator.trafficPattern.roadTravelTimeToEndIntersection(startedSearch, loc), simulator, fleetManager);
		this.loc = loc;
		this.startSearchTime = startedSearch;
	}

	@Override
	Event trigger() throws Exception {
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "******** AgentEvent id = " + id+ " triggered at time " + time, this);
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "Loc = " + loc, this);

		switch (state) {
			case INTERSECTION_REACHED:
				navigate();
				break;
			case PICKING_UP:
				pickup();
				break;
			case DROPPING_OFF:
				dropOff();
				break;
		}
		return this;
	}

	boolean hasPickupRes() {
		return isPickup;
	}

	void assignTo(ResourceEvent resourceEvent) {
		this.assignedResource = resourceEvent;
	}

	void abortResource() {
		assignedResource = null;
		isPickup = false;
		state = State.INTERSECTION_REACHED;
	}

	void navigate() throws Exception {
		// TODO: assert with DistanceLocationOnRoad
		//assert loc.travelTimeFromStartIntersection == loc.road.travelTime : "Agent not at an intersection.";

		if (isArrivingPickupLoc()) {
			long travelTimeToPickupLoc = (long)simulator.trafficPattern.roadTravelTimeFromStartIntersection(time, assignedResource.pickupLoc);
			long nextEventTime = time + travelTimeToPickupLoc;
			update(nextEventTime, assignedResource.pickupLoc, State.PICKING_UP);
			return;
		}

		if (isArrivingDropOffLoc()) {
			long travelTimeToDropoffLoc = (long)simulator.trafficPattern.roadTravelTimeFromStartIntersection(time, assignedResource.dropoffLoc);
			long nextEventTime = time + travelTimeToDropoffLoc;
			update(nextEventTime, assignedResource.dropoffLoc, State.DROPPING_OFF);
			return;
		}


		Intersection nextIntersection;
		if (isPickup && assignedResource != null) {
			nextIntersection = fleetManager.onReachIntersectionWithResource(id, time, simulator.agentCopy(loc), assignedResource.copyResource());
		} else {
			nextIntersection = fleetManager.onReachIntersection(id, time, simulator.agentCopy(loc));
		}

		if (nextIntersection == null) {
			throw new Exception("agent.move() did not return a next location");
		}

		if (!loc.link.road.to.isAdjacent(nextIntersection)) {
			throw new Exception("move not made to an adjacent location");
		}

		// set location and time of the next trigger
		Road nextRoad = loc.link.road.to.roadTo(nextIntersection);
		DistanceLocationOnLink nextLocation = new DistanceLocationOnLink(nextRoad.links.get(nextRoad.links.size()-1), nextRoad.length);
		long travelTimeToNextLocation = (long)(simulator.trafficPattern.roadTravelTimeToEndIntersection(time, nextLocation));
		update(time + travelTimeToNextLocation, nextLocation, State.INTERSECTION_REACHED);

		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "Move to " + nextRoad.to, this);
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "Next trigger time = " + time, this);
	}

	private boolean isArrivingPickupLoc() {
		return !isPickup && assignedResource != null && assignedResource.pickupLoc.link.road.from.equals(loc.link.road.to);
	}

	private boolean isArrivingDropOffLoc() {
		return isPickup && assignedResource != null && assignedResource.dropoffLoc.link.road.from.equals(loc.link.road.to);
	}

	/*
	 * The handler of a pick up event.
	 */
	private void pickup() {
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "Pickup at " + loc, this);

		isPickup = true;
		long searchTime = time - startSearchTime;

		// TODO I believe this should be the time between when it become available and when it's picked-up.
		//    This is not what is being computed here.
		long waitTime = assignedResource.time - assignedResource.availableTime;

		simulator.totalAgentSearchTime += searchTime;
		simulator.totalResourceWaitTime += waitTime;

		assignedResource.pickup(this, time);

		// move to the end intersection of the current road
		long travelTimeToEndIntersection = (long)(simulator.trafficPattern.roadTravelTimeToEndIntersection(time, loc));
		long nextEventTime = time + travelTimeToEndIntersection;
		DistanceLocationOnLink nextLoc = new DistanceLocationOnLink(loc.link.road.links.get(loc.link.road.links.size()-1), loc.link.length);
		update(nextEventTime, nextLoc, State.INTERSECTION_REACHED);
	}

	/*
	 * The handler of a drop off event.
	 */
	private void dropOff() {
		startSearchTime = time;
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "Dropoff at " + loc, this);

		isPickup = false;
		assignedResource.dropOff(time);
		assignedResource = null;

		// move to the end intersection of the current road
		long travelTimeToEndIntersection = (long)(simulator.trafficPattern.roadTravelTimeToEndIntersection(time, loc));
		long nextEventTime = time + travelTimeToEndIntersection;
		DistanceLocationOnLink nextLoc = new DistanceLocationOnLink(loc.link.road.links.get(loc.link.road.links.size()-1), loc.link.length);
		update(nextEventTime, nextLoc, State.INTERSECTION_REACHED);
	}

	private void update(long time, DistanceLocationOnLink loc, State state) {
		this.time = time;
		this.loc = loc;
		this.state = state;
	}
}

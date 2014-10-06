package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/* Author: Vijay Jayaram (vijayj) */
public class CarButtonControl extends Controller {
	
    /***************************************************************************
     * Declarations
     **************************************************************************/
	
    //local physical state
    private ReadableCarCallPayload localCarCall;
    private WriteableCarLightPayload localCarLight;
    
    //network interface
    private WriteableCanMailbox networkCarLightOut;
    // translator for the car light message -- this is a generic translator
    private BooleanCanPayloadTranslator mCarLight;
    
    // network interface
    private WriteableCanMailbox networkCarCall;
    private BooleanCanPayloadTranslator mCarCall;

    //received door closed message
    private ReadableCanMailbox networkDoorClosedLeft;
    private ReadableCanMailbox networkDoorClosedRight;
    
    //translator for the AtFloor message -- this translator is specific
    //to this message and is provided elevatormodules package
    private ReadableCanMailbox networkAtFloor;
    private AtFloorCanPayloadTranslator mAtFloor;
    
    //translator for the DesiredFloor message -- this translator is specific
    //to this message and is provided elevatormodules package
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final int floor;
    
    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_IDLE,
        STATE_CAR_CALLED
    }
    
    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_IDLE;

    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
    public CarButtonControl(SimTime period, int floor, Hallway hallway, boolean verbose) {
        //call to the Controller superclass constructor is required
        super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway), verbose);
        
        //stored the constructor arguments in internal state
        this.period = period;
        this.hallway = hallway;
        this.floor = floor;
        
        /* 
         * The log() method is inherited from the Controller class.  It takes an
         * array of objects which will be converted to strings and concatenated
         * only if the log message is actually written.  
         */
        log("Created " + getReplicationName() + " with period = ", period);

        /* 
         * Create readable payloads and translators for all messages in input interface
         * 		- mAtFloor[f,b,d], mDesiredFloor, mDoorClosed[b,r], HallCall[f,b,d]
         */
       
        // mAtFloor
        networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
        
        // mDesiredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        
        // mDoorClosedLeft, mDoorClosedRight
        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        
        // HallCall
        localCarCall = CarCallPayload.getReadablePayload(floor, hallway);

        // Register input messages periodically
        canInterface.registerTimeTriggered(networkAtFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDoorClosedLeft);
        canInterface.registerTimeTriggered(networkDoorClosedRight);
        physicalInterface.registerTimeTriggered(localCarCall);
        
        
        /* 
         * Create writeable payloads for all messages in output interface
         * 		- CarLight[f,b,d], mCarLight[f,b,d], mCarCall[f,b,d]
         */
        
        // HallLight
        localCarLight = CarLightPayload.getWriteablePayload(floor, hallway);
        
        // mCarLight
        networkCarLightOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
        mCarLight = new BooleanCanPayloadTranslator(networkCarLightOut);
        
        // mCarCall
        networkCarCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
        mCarCall = new BooleanCanPayloadTranslator(networkCarCall);
        
        // Send/broadcast output messages periodically
        physicalInterface.sendTimeTriggered(localCarLight, period);
        canInterface.sendTimeTriggered(networkCarLightOut, period);
        canInterface.sendTimeTriggered(networkCarCall, period);

        // Start the periodic timer
        timer.start(period);
    }

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
        switch(state) {
            case STATE_IDLE:
            	localCarLight.set(false);
            	mCarLight.set(false);
            	mCarCall.set(false);
            	//#transition CBC.1
            	if (localCarCall.pressed()) {
            		newState = State.STATE_CAR_CALLED;
            	}
                break;
            case STATE_CAR_CALLED:
            	localCarLight.set(true);
            	mCarLight.set(true);
            	mCarCall.set(true);
            	//#transition CBC.2
            	if (mAtFloor.getValue() && mDesiredFloor.getFloor() == floor &&
            			mDesiredFloor.getHallway().equals(hallway)) {
            		newState = State.STATE_IDLE;
            	}
                break;
            default:
            	throw new RuntimeException("State " + state + " was not recognized.");
        }
        
    	log(state.toString() + " -> " + newState.toString());
        state = newState;

        timer.start(period);
	}
	
	private String getReplicationName() {
		return "CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway);
	}

}
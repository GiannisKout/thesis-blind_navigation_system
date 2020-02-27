import com.google.android.gms.maps.model.LatLng;

public class PolyPointObject {

    // Constructor where listener events are ignored
    public PolyPointObject() {
        // set null or default listener or accept as argument to constructor
        this.polyPointListener = null;
    }

    // Assign the listener implementing events interface that will receive the events
    public void setPolyPointListener(PolyPointListener listener) {
        this.polyPointListener = listener;
    }

    // Step 1 - This interface defines the type of messages I want to communicate to my owner
    public interface PolyPointListener {
        // These methods are the different events and
        // need to pass relevant arguments related to the event triggered
        public void onPolyPointReached(LatLng nextPoint);
        // or when data has been loaded

    }

    private PolyPointListener polyPointListener;
}

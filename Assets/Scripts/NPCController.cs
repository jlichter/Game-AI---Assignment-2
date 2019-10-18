using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class NPCController : MonoBehaviour {
    // Store variables for objects
    private SteeringBehavior ai;    // Put all the brains for steering in its own module
    private Rigidbody rb;           // You'll need this for dynamic steering

    // For speed 
    public Vector3 position;        // local pointer to the RigidBody's Location vector
    public Vector3 velocity;        // Will be needed for dynamic steering

    // For rotation
    public float orientation;       // scalar float for agent's current orientation
    public float rotation;          // Will be needed for dynamic steering

    public float maxSpeed;          // what it says

    public int phase;               // use this to control which "phase" the demo is in

    private Vector3 linear;         // The resilts of the kinematic steering requested
    private float angular;          // The resilts of the kinematic steering requested

    public Text label;              // Used to displaying text nearby the agent as it moves around
    LineRenderer line;              // Used to draw circles and other things

    [Header("Our variables")]
    public bool isPlayer;
    public PlayerController player;


    private void Start() {
        ai = GetComponent<SteeringBehavior>();
        rb = GetComponent<Rigidbody>();
        line = GetComponent<LineRenderer>();
        position = rb.position;
        orientation = transform.eulerAngles.y;
        
        
    }

    /// <summary>
    /// Depending on the phase the demo is in, have the agent do the appropriate steering.
    /// 
    /// </summary>
    void FixedUpdate() {
        switch (phase) {
            case 0: /* note = > NOT MAPPED : DEFAULT WHILE WAITING FOR INPUT */
                if(label) {
                    label.text = name.Replace("(Clone)", "") + "\nWaiting for input";
                }
          
                break;

            case 1: /*note =>  PURSUE (testing with wall avoidance / collision detection too) */
                if (label) {
                    // replace "First algorithm" with the name of the actual algorithm you're demoing
                    // do this for each phase
                    label.text = name.Replace("(Clone)","") + "\nAlgorithm: Pursue algorithm(s)"; 
                }
                Behavior aiAvoidP = new Behavior(3f, 0f, ai.WallAvoidance());
                Behavior aiPursue = new Behavior(1f, 0f, ai.Pursue());
            
                linear = (aiPursue.weight * aiPursue.behavior) + (aiAvoidP.weight * aiAvoidP.behavior);


                break;

            case 2: /* note => EVADE (testing wall avoidance/ collision detection too) */
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nAlgorithm: Evade algorithm(s)";
                }
                Behavior aiAvoidE = new Behavior(3f, 0f, ai.WallAvoidance());
                Behavior aiEvade = new Behavior(0.8f, 0f, ai.Evade());
      
                linear = (aiEvade.weight * aiEvade.behavior) + (aiAvoidE.weight * aiAvoidE.behavior);
                
               // linear = 
                break;
            case 3: /* note => ARRIVE */
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nAlgorithm: Arrive algorithm";
                }
                Behavior aiAvoidA = new Behavior(3f, 0f, ai.WallAvoidance());
                Behavior aiArrive = new Behavior(1f, 0f, ai.Pursue());
                linear = (aiArrive.weight * aiArrive.behavior) + (aiAvoidA.weight * aiAvoidA.behavior);
                linear = ai.Arrive();

                break;
            case 4: /* note => WANDER */
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nAlgorithm: Wander algorithm";
                }
                Vector3 wallAvoid = ai.WallAvoidance();
                
                linear = ai.Wander() + 4 * wallAvoid;
                angular = 2*ai.LookWhereYoureGoing();
                
                DrawCircle(ai.wanderCircleCenter, ai.wanderRadius);
                break;
            case 5: // note ==> PURSUING PLAYER WITH COLLISION PREDICTION
                 if (label) {
                    label.text = name.Replace("(Clone)", "") + "chasing the player";
                }
                Vector3 showPrediction = ai.CollisionPrediction();
                // linear = ai.whatever();  -- replace with the desired calls
                // angular = ai.whatever();
                break;
        }
        if (isPlayer) {
            linear = player.GetComponent<PlayerController>().velocity;

        }

        update(linear, angular, Time.deltaTime);
        if (label) {
            label.transform.position = Camera.main.WorldToScreenPoint(this.transform.position);
        }
    }

    private void update(Vector3 steeringlin, float steeringang, float time) {

        if (!isPlayer) {
            // Update the orientation, velocity and rotation
            orientation += rotation * time;
            velocity += steeringlin * time;
            rotation += steeringang * time;

            if (velocity.magnitude > maxSpeed) {
                velocity.Normalize();
                velocity *= maxSpeed;
            }

            rb.AddForce(velocity - rb.velocity, ForceMode.VelocityChange);
            position = rb.position;
            rb.MoveRotation(Quaternion.Euler(new Vector3(0, Mathf.Rad2Deg * orientation, 0)));
        } else {
            position = player.GetComponent<PlayerController>().position;
            velocity = player.GetComponent<PlayerController>().velocity;
        }

    }

    // <summary>
    // The next two methods are used to draw circles in various places as part of demoing the
    // algorithms.

    /// <summary>
    /// Draws a circle with passed-in radius around the center point of the NPC itself.
    /// </summary>
    /// <param name="radius">Desired radius of the concentric circle</param>
    public void DrawConcentricCircle(float radius) {
        line.positionCount = 51;
        line.useWorldSpace = false;
        float x;
        float z;
        float angle = 20f;

        for (int i = 0; i < 51; i++) {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * radius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * radius;

            line.SetPosition(i, new Vector3(x, 0, z));
            angle += (360f / 51);
        }
    }

    /// <summary>
    /// Draws a circle with passed-in radius and arbitrary position relative to center of
    /// the NPC.
    /// </summary>
    /// <param name="position">position relative to the center point of the NPC</param>
    /// <param name="radius">>Desired radius of the circle</param>
    public void DrawCircle(Vector3 position, float radius) {
        line.positionCount = 51;
        line.useWorldSpace = true;
        float x;
        float z;
        float angle = 20f;

        for (int i = 0; i < 51; i++) {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * radius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * radius;

            line.SetPosition(i, new Vector3(x, 0, z)+position);
            angle += (360f / 51);
        }
    }

    public void DestroyPoints() {
        if (line) {
            line.positionCount = 0;
        }
    }
    struct Behavior {

        public float weight;
        public float Rot;
        public Vector3 behavior;

        public Behavior(float w, float r, Vector3 b) {
            weight = w;
            Rot = r;
            behavior = b;
        }
    }

}

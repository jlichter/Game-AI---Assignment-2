﻿using System.Collections;
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
    public bool isHouse;
    public bool isHunter;
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
                    label.text = name.Replace("(Clone)", "");
                }
          
                break;

            case 1: /*note =>  PURSUE (testing with wall avoidance / collision detection too) */
                if (label) {
                    // replace "First algorithm" with the name of the actual algorithm you're demoing
                    // do this for each phase
                    label.text = name.Replace("(Clone)","") + "\nAlgorithm: Pursue algorithm(s)"; 
                }
                Behavior aiAvoidP = new Behavior(3f, 0f, ai.WallAvoidance());
                Behavior aiPursue = new Behavior(0.9f, 0f, ai.Pursue());
            
                linear = (aiPursue.weight * aiPursue.behavior) + (aiAvoidP.weight * aiAvoidP.behavior);
                angular = ai.Face();
                //angular = 0f;

                break;

            case 2: /* note => EVADE (testing wall avoidance/ collision detection too) */
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nAlgorithm: Evade algorithm(s)";
                }
                Behavior aiAvoidE = new Behavior(3f, 0f, ai.WallAvoidance());
                Behavior aiEvade = new Behavior(0.8f, 0f, ai.Evade());
      
                linear = (aiEvade.weight * aiEvade.behavior) + (aiAvoidE.weight * aiAvoidE.behavior);
                angular = ai.LookWhereYoureGoing();
               // linear = 
                break;
            case 3: /* note => ARRIVE */
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nAlgorithm: Arrive algorithm";
                }
                Behavior aiAvoidA = new Behavior(4f, 0f, ai.WallAvoidance());
                Behavior aiArrive = new Behavior(0.9f, 0f, ai.Arrive());
                /*
                if (ai.WallAvoidance() != Vector3.zero) {
                    linear = aiAvoidA.weight * aiAvoidA.behavior;
                } else {
                    linear = aiArrive.weight * aiArrive.behavior;
                }
                */
                if (isHunter) {
                    linear = 2 * ai.Arrive();
                    angular = 3 * ai.LookWhereYoureGoing();
                } else {
                    linear = ai.Arrive();
                    //   linear = (aiArrive.weight * aiArrive.behavior) + (aiAvoidA.weight * aiAvoidA.behavior);
                    angular = 3 * ai.LookWhereYoureGoing();
                }

                break;
            case 4: /* note => WANDER */
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nAlgorithm: Wander algorithm";
                }
                //  Vector3 newLinear;
                Vector3 Wandering;
                //angular = ai.Wander(out Wandering);
                //   linear = newLinear;
               // if(phase == 2) {
                 //   Debug.Log("whats up?");
                //}
                //Vector3 wallAvoid = ai.WallAvoidance();
                /*
                if (wallAvoid != Vector3.zero) {
                    linear = wallAvoid;
                } else {
                    linear = Wandering;
                }
                */
             //   linear = Wandering;
                Vector3 wallAvoid = ai.WallAvoidance();

                //linear = (0.9f * ai.Wander()) + (5f * wallAvoid);
                linear = ai.Wander();
                angular = 3 * ai.LookWhereYoureGoing();

                DrawCircle(ai.wanderCircleCenter, ai.wanderRadius);
                // linear = (0.9f * Wandering) + (5f * wallAvoid);
                //   Debug.Log("here is lookWhereYoureGoing" + ai.LookWhereYoureGoing());
                /*
                if (!float.IsNaN(angular)) {
                    angular = 2 * ai.LookWhereYoureGoing();
                }
                */
                angular = 2*ai.LookWhereYoureGoing();
                
              //  DrawCircle(ai.wanderCircleCenter, ai.wanderRadius);
               // Debug.Log("still here!");
                break;
            case 5: // note ==> PURSUING PLAYER WITH COLLISION PREDICTION
                 if (label) {
                    label.text = name.Replace("(Clone)", "") + "chasing the player";
                }
                Vector3 showPrediction = ai.CollisionPrediction();
                // linear = ai.whatever();  -- replace with the desired calls
                // angular = ai.whatever();
                break;
            case 6:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "Following the path";
                }
                linear = ai.followPath(); // + 3 * ai.WallAvoidance();
                angular = ai.LookWhereYoureGoing();
                break;
            case 7:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "Stopped";
                }
                linear = ai.stop();
                break;
            case 8: // is house 
                isHouse = true;
                break;
        }
        if (isPlayer) {
            linear = player.GetComponent<PlayerController>().velocity;

        }
        if (!isHouse) {
            update(linear, angular, Time.deltaTime);
        }
        
        if (label) {
            label.transform.position = Camera.main.WorldToScreenPoint(this.transform.position);
        }
    }

    private void update(Vector3 steeringlin, float steeringang, float time) {

        if (!isPlayer) {
            // Update the orientation, velocity and rotation
            if (!isHouse) {
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
            } 
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

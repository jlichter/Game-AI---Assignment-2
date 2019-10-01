﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is the place to put all of the various steering behavior methods we're going
/// to be using. Probably best to put them all here, not in NPCController.
/// </summary>

public class SteeringBehavior : MonoBehaviour {

    // The agent at hand here, and whatever target it is dealing with
    public NPCController agent;
    public NPCController target;

    // Below are a bunch of variable declarations that will be used for the next few
    // assignments. Only a few of them are needed for the first assignment.

    // For pursue and evade functions
    public float maxPrediction;
    public float maxAcceleration;

    // For arrive function
    public float maxSpeed;
    public float targetRadiusL;
    public float slowRadiusL;
    public float timeToTarget;

    // For Face function
    public float maxRotation;
    public float maxAngularAcceleration;
    public float targetRadiusA;
    public float slowRadiusA;

    // For wander function
    public float wanderOffset;
    public float wanderRadius;
    public float wanderRate;
    private float wanderOrientation;

   

    // Holds the path to follow
    public GameObject[] Path;
    public int current = 0;

    [Header("Our Variables")]
    public float pred;
    [Header("For wander")]
    public Vector3 wanderCircleCenter;
    [Header("Ray 'sensors'")]
    public float raysLength = 20f; // holds the distance to look ahead for a collision 
    public float frontRayPosition = 0.5f;

    // (jessie) for collision avoidance, need list of potential targets 
   // public GameObject[] targets;

    protected void Start() {
        agent = GetComponent<NPCController>();
        wanderOrientation = agent.orientation;
    }

    public Vector3 Seek() {
        // Get the direction to the target
        Vector3 direction = target.position - agent.position;
        // The velocity is along this direction, at full speed
        direction.Normalize();
        direction *= maxAcceleration;
        // Output the steering
        return direction;
    }

    public Vector3 Flee()
    {
        return new Vector3(0f, 0f, 0f);
    }

    /*
    // Calculate the target to pursue
    public Vector3 Pursue() {
        return new Vector3(0f, 0f, 0f);
    }
    */

    public float Face()
    {
        return 0f;
    }


    // ETC.

/* todo: "sensors" for raycasts, finish putting the other ones in 
 and make the agent avoid the obstacles. then, make the chasing agent
 avoid them too . once u get that to work, see about making algos 
 "more intelligent* / 

    /*
 * getSteering() calculates a surrogate target
 * and returns the target's position
 * (from assignment 1)
 */
    // Jessie 
    public Vector3 getSteering() {
        // work out the distance to target  
        Vector3 direction = target.position - agent.position;
        float distance = direction.magnitude;

        // work out our current speed
        float speed = agent.velocity.magnitude;

        // for our end prediction
        float prediction;

        // check if speed is too small to give a reasonable prediction time 
        if (speed <= distance / maxPrediction) {
            prediction = maxPrediction;
        } else {
            prediction = distance / speed;
        }
        pred = prediction;
        // get target's new position 
        Vector3 targetPos = target.position + target.velocity * prediction;

        return targetPos; // return the position
    }
    /*
     * Pursue() calls getSteering() and calculates the
        direction from the character to the target and 
        requests a velocity along this line
        (from assignment 1)
    */
    public Vector3 Pursue() {

        // call to getSteering()
        Vector3 targetPosition = getSteering();
        // get the direction to the target 
        Vector3 steering = targetPosition - agent.position;
        // the velocity is along this direction, at full speed 
        steering.Normalize();
        steering *= maxAcceleration;
        // for clarity
        agent.DrawCircle(target.position + target.velocity * pred, 0.4f);
        //output the steering
        return steering;

    }
    /*
     * Evade() calls getSteering() and calculates the
        direction from the character to the target and 
        requests a velocity in the opposite direction
        (from assignment 1)
    */
    public Vector3 Evade() {
        // call to getSteering()
        Vector3 targetPosition = getSteering();
        // get the direction away from the target
        Vector3 steering = agent.position - targetPosition;
        // the velocity is along this direction, at full speed 
        steering.Normalize();
        steering *= maxAcceleration;
        // for clarity
        agent.DrawCircle(target.position + target.velocity * pred, 0.4f);
        //output the steering
        return steering;

    }
 /* 
  * Arrive() causes the agent to move towards the target, like pursue.
  *     but unlike pursue, Arrive() causes the agent to slow down as it arrives
 *      exactly at the right location.
 */
    public Vector3 Arrive() {

        // Create the structure to hold our output
        Vector3 steering;

        // get the direction to the target 
        Vector3 direction = target.position - agent.position;
        float distance = direction.magnitude;
        float targetSpeed;
        // Check if we are there, return no steering

        //  If we are outside the slowRadius, then go max speed
        if (distance < targetRadiusL) {
            //return Vector3.zero;
            targetSpeed = 0;
        } else if (distance > slowRadiusL) {
            targetSpeed = maxSpeed;
        } // Otherwise calculate a scaled speed
          else {
            targetSpeed = (maxSpeed * distance) / slowRadiusL;
        }

        // The target velocity combines speed and direction
        Vector3 targetVelocity = direction;
        targetVelocity.Normalize();
        targetVelocity *= targetSpeed;

        // Acceleration tries to get to the target velocity
        steering = targetVelocity - agent.velocity;
        steering = steering / timeToTarget;

        // Check if the acceleration is too fast
        if (steering.magnitude > maxAcceleration) {
            steering.Normalize();
            steering *= maxAcceleration;
        }

        // output the steering 
        return steering;
    }

    /* 
     * Align() tries to match the orientation of the character with that of the target. 
     * It pays no attention to the position or velocity of the character or target. 
     * (from assignment 1)
     * todo : needs to be improved (visuals)
    */
    public float Align() {

        // Create the structure to hold our output
        float steering_angular;

        // Get the naive direction to the target
        float rotation = target.orientation - agent.orientation;

        // map the result to the (-pi,pi) interval 
        while (rotation > Mathf.PI) {
            rotation -= 2 * Mathf.PI;
        }
        while (rotation < -Mathf.PI) {
            rotation += 2 * Mathf.PI;
        }
        float rotationSize = Mathf.Abs(rotation);

        // Check if we are there, return no steering
        if (rotationSize < targetRadiusA) {
            agent.rotation = 0;
        }

        // Otherwise calculate a scaled rotation 
        float targetRotation;
        if (rotationSize > slowRadiusA) {
            targetRotation = maxRotation;
        } else {
            targetRotation = (maxRotation * rotationSize) / slowRadiusA;
        }

        // The final target rotation combines
        // speed (already in the variable) and direction
        targetRotation *= (rotation / rotationSize);

        // Acceleration tries to get to the target rotation
        steering_angular = targetRotation - agent.rotation;
        steering_angular = steering_angular / timeToTarget;

        // Check if the acceleration is too great
        float angularAcceleration;
        angularAcceleration = Mathf.Abs(steering_angular);
        if (angularAcceleration > maxAngularAcceleration) {
            steering_angular = steering_angular / angularAcceleration;
            steering_angular = steering_angular / angularAcceleration;
            steering_angular *= maxAngularAcceleration;
        }
        // 
        // output the steering 
        return steering_angular;

    }
    /* direction the character is facing as a 3d Vector */
    private Vector3 orientationVector(float angle) {
        return new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle));
    }

    /*
     * Wander() controls a character moving aimlessly about
     * (from assignment 1)
     */
    public Vector3 Wander() {

        // Update the wander orientation
        wanderOrientation += (Random.value - Random.value) * wanderRate;
        // Calculate the combined target orientation
        float target_orientation = agent.orientation + wanderOrientation;
        // Calculate the center of the wander circle
        Vector3 target_position = agent.position + orientationVector(agent.orientation) * wanderOffset;
        wanderCircleCenter = target_position;
        // Calculate the target location
        target_position += orientationVector(target_orientation) * wanderRadius;
        // Delegate to face
        Vector3 steering = target_position - agent.position;
        if (target == null) {
            target = new NPCController();
        }
        target.orientation = Mathf.Atan2(steering.x, steering.z);
        // Now return the linear acceleration to be at full
        // acceleration in the direction of the orientation
        return steering.normalized * maxAcceleration;
    }

    public bool CollisionDetection(Vector3 rayStart, RaycastHit hitPoint, out Vector3 collisionPos, out Vector3 collisionNorm) {

        collisionPos = new Vector3(0f, 0f, 0f); 
        collisionNorm = new Vector3(0f, 0f, 0f);
        Vector3 agentVelocity = agent.velocity;
        agentVelocity.Normalize();
        // see if sphere cast detects collision ahead of player travel
        // if so, set the collision position and the collision normal 
        if (Physics.SphereCast(rayStart, 0.1f, agentVelocity, out hitPoint, raysLength)) {
            Debug.Log("here!");
            collisionPos = hitPoint.point;
            collisionNorm = hitPoint.normal;
            Debug.DrawLine(rayStart, hitPoint.point);
            return true; // if collision detected, return true 
        }

        return false; // else, no collision, return false 

    }

    public Vector3 WallAvoidance() {

        // holds the information about 
        RaycastHit hit = new RaycastHit();
        // holds the minimum distance to a wall 
        float avoidDistance = 5f;
        // calculate the collision ray vector 
        Vector3 rayStartPos = agent.position;
        rayStartPos.z += frontRayPosition;
        Vector3 cpTemp = new Vector3(0f, 0f, 0f);
        Vector3 cnTemp = new Vector3(0f, 0f, 0f);

        // find the collision
        if (CollisionDetection(rayStartPos, hit, out cpTemp, out cnTemp)) {

            Vector3 collisionPosition = cpTemp;
            Vector3 collisionNormal = cnTemp;
            Vector3 newPosition = -collisionPosition + collisionNormal * avoidDistance;
            // Get the direction to the target
            Vector3 direction = newPosition - agent.position;
            // The velocity is along this direction, at full speed
            direction.Normalize();
            direction *= maxAcceleration;
            return direction;

        } else {
            //if no collision, do nothing 
            return new Vector3(0f,0f,0f);

        }

    }
    // todo collision prediction
    // todo collision detection
    // todo chase the player
    // todo more intellgient wander
    // todo more intelligent behavior overall

    public float LookWhereYoureGoing() {
        // Create the structure to hold our output
        float steering_angular;
        float direction = Mathf.Atan2(agent.velocity.x, agent.velocity.z);
        // Get the naive direction to the target
        float rotation = direction - agent.orientation;
        
        // map the result to the (-pi,pi) interval 
        while (rotation > Mathf.PI) {
            rotation -= 2 * Mathf.PI;
        }
        while (rotation < -Mathf.PI) {
            rotation += 2 * Mathf.PI;
        }
        float rotationSize = Mathf.Abs(rotation);

        // Check if we are there, return no steering
        if (rotationSize < targetRadiusA) {
            agent.rotation = 0;
        }

        // Otherwise calculate a scaled rotation 
        float targetRotation;
        if (rotationSize > slowRadiusA) {
            targetRotation = maxRotation;
        }
        else {
            targetRotation = (maxRotation * rotationSize) / slowRadiusA;
        }

        // The final target rotation combines
        // speed (already in the variable) and direction
        targetRotation *= (rotation / rotationSize);

        // Acceleration tries to get to the target rotation
        steering_angular = targetRotation - agent.rotation;
        steering_angular = steering_angular / timeToTarget;

        // Check if the acceleration is too great
        float angularAcceleration;
        angularAcceleration = Mathf.Abs(steering_angular);
        if (angularAcceleration > maxAngularAcceleration) {
            steering_angular = steering_angular / angularAcceleration;
            steering_angular = steering_angular / angularAcceleration;
            steering_angular *= maxAngularAcceleration;
        }
        // 
        // output the steering 
        return steering_angular;
    }

    
    private void Rays() {



        
    }

}

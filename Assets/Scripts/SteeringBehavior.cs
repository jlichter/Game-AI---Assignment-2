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
    public PlayerController playerTarget;
    [Header("For wander")]
    public Vector3 wanderCircleCenter;
    [Header("Ray 'sensors'")]
    public float frontRayPosition;
    [Header("Collision Detection")]
    public Transform centerDetector; // for detecting collisions in front 
    public Transform rightAngleDetector; // for detecting collisions to right
    public Transform leftAngleDetector; // for detecting collisions to left 

    public Vector3 collisionPosition;
    public Vector3 collisionNormal;
    public bool metEnd; // bool to check if hunter arrived at house for end of game 

    // (jessie) for collision avoidance, need list of potential targets 
   // public GameObject[] targets;

    protected void Start() {

        agent = GetComponent<NPCController>();
        wanderOrientation = agent.orientation;
        frontRayPosition = agent.position.z;
      //  maxPrediction = 1f;

        // initialize our "detectors" for collision avoidance
        if(centerDetector == null) {
            centerDetector = agent.transform;
        }
        if(rightAngleDetector == null) {
            rightAngleDetector = agent.transform;
        }
        if (leftAngleDetector == null) {
            leftAngleDetector = agent.transform;
        }

        metEnd = false; // initially, we are not at end of game 

    }


    public float getOrientation(float currentOrientation, Vector3 velocity) {

        float newOrientation;
        // make sure we have a velocity 
        if(velocity != Vector3.zero) {
            // calculate orientation using arc tangent of the velocity components
            // return 
            newOrientation = Mathf.Atan2(-velocity.x, velocity.z);
            return newOrientation;
        } else {
            // otherwise use the current orientation 
            return currentOrientation;
        }

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


    /*
    // Calculate the target to pursue
    public Vector3 Pursue() {
        return new Vector3(0f, 0f, 0f);
    }
    */

    public float Face() {

        // Create the structure to hold our output
        float steering_angular;

        // Get the naive direction to the target
        Vector3 direction = target.position - agent.position;

        float rotation = Mathf.Atan2(direction.x, direction.z) - agent.orientation;

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


    // ETC.




    Vector3 Flee() {
        // work out the distance away from target  
        Vector3 direction = agent.position - target.position;
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
        //pred = prediction;
        // get target's new position 
        Vector3 targetPos = target.position + target.velocity * prediction;

        return targetPos; // return the position
    }

    public Vector3 Pursue() {
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
        // visual 
        agent.DrawCircle(target.position + target.velocity * prediction, 0.3f);
        // Create the structure to hold our output
        Vector3 steering = (target.position + target.velocity * prediction) - agent.position;
        // Give full acceleration along this direction
        steering.Normalize();
        steering *= maxAcceleration;
        //output the steering
        return steering;
    }

    public Vector3 Evade() {

        // work out the distance to the target  
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
        // visual 
        agent.DrawCircle(target.position + target.velocity * prediction, 0.3f);
        // Create the structure to hold our output
        Vector3 steering = agent.position - (target.position + target.velocity * prediction);
        // Give full acceleration along this direction
        steering.Normalize();
        steering *= maxAcceleration;
        //output the steering
        return steering;
    }
 /* 
  * Arrive() causes the agent to move towards the target, like pursue.
  *     but unlike pursue, Arrive() causes the agent to slow down as it arrives
 *      exactly at the right location.
 */
    public Vector3 Arrive() {
        // holds collision detector for arrive
        RaycastHit hit;
        // Holds the minimum distance to a wall (i.e., how far
        // to avoid collision) should be greater than the
        // radius of the character.
        float avoidDistance = 25f;
        // Holds the distance to look ahead for a collision
        // (i.e., the length of the collision ray)
        float raysLength = 5f;
        // Calculate the collision ray vector
        Vector3 forwardRay = agent.velocity;
        forwardRay.y = 0f;
        forwardRay.Normalize();
        // shoot the ray from the character's current position
        Vector3 rayStartPos = agent.position;
        // holds the collision position and normal if collision detected

        // Find the collision

        if (Physics.SphereCast(rayStartPos, 1f, forwardRay, out hit, raysLength)) {

            Debug.DrawRay(rayStartPos, hit.point, Color.green);

            if (target == null || hit.transform != target.transform) {
                Vector3 newTarget = hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
                dir.Normalize();
                dir *= 2;
                dir *= maxAcceleration;
                return dir;

            }
        }
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
     //   agent.DrawCircle(targetVelocity, 0.4f);
        // Acceleration tries to get to the target velocity
        steering = targetVelocity - agent.velocity;
        steering /= timeToTarget;

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
    private Vector3 OrientationVector(float angle) {
        Vector3 orientationVec = new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle));
       // Debug.Log("orientation vec" + orientationVec);
        return orientationVec;
        
    }

    /*
     * Wander() controls a character moving aimlessly about
     * (from assignment 1)
     */
     /*
    public float Wander(out Vector3 linear) {

        // holds collision detector for wander 
        RaycastHit hit;
        // Holds the minimum distance to a wall (i.e., how far
        // to avoid collision) should be greater than the
        // radius of the character.
        float avoidDistance = 15f;
        // Holds the distance to look ahead for a collision
        // (i.e., the length of the collision ray)
        float raysLength = 4f;
        // Calculate the collision ray vector
        Vector3 forwardRay = agent.velocity;
        //forwardRay.y = 0f;
        forwardRay.Normalize();
        // shoot the ray from the character's current position
        Vector3 rayStartPos = agent.position;
        // holds the collision position and normal if collision detected

        // Find the collision

        if (Physics.SphereCast(rayStartPos, 1f, forwardRay, out hit, raysLength)) {

            Debug.DrawRay(rayStartPos, hit.point, Color.green);

            if (target == null || hit.transform != target.transform) {
                Vector3 newTarget = -hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
                dir.Normalize();
                dir *= maxAcceleration;
                linear = dir;
                return LookWhereYoureGoing();

            }
        }
        // Update the wander orientation
        wanderOrientation += (Random.value - Random.value) * wanderRate;
        // Calculate the combined target orientation
        float target_orientation = agent.orientation + wanderOrientation;
        // Calculate the center of the wander circle
        Vector3 target_position = agent.position + wanderOffset * OrientationVector(agent.orientation);
        agent.DrawCircle(target_position, wanderRadius);
        // wanderCircleCenter = target_position;
        // Calculate the target location
        target_position += wanderRadius * OrientationVector(target_orientation);
         
        // Delegate to face
        Vector3 steering = target_position - agent.position;
        // Check for a zero direction, and make no change if so
        if (steering.magnitude == 0) {
            linear = Vector2.zero;
            return 0;
        }
        // Put the target orientation together (face, then align)
        float orientation = Mathf.Atan2(-steering.x, steering.z);
        // Get the naive direction to the target
        float rotation = orientation - agent.orientation;
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
        // The final target rotation combines speed (already in the variable) and direction
        targetRotation *= rotation / rotationSize;

        float angular = targetRotation - agent.rotation;
        angular /= timeToTarget;
        // Check if the acceleration is too great
        float angularAcceleration = Mathf.Abs(angular);
        if (angularAcceleration > maxAngularAcceleration) {
            angular /= angularAcceleration;
            angular *= maxAngularAcceleration;
        }
        // Now return the linear acceleration to be at full
        // acceleration in the direction of the orientation
        linear = maxAcceleration * OrientationVector(agent.orientation);
 
        return angular;
    }
    */
    public Vector3 Wander() {
        // holds collision detector for wander 
        RaycastHit hit;
        // Holds the minimum distance to a wall (i.e., how far
        // to avoid collision) should be greater than the
        // radius of the character.
        float avoidDistance = 15f;
        // Holds the distance to look ahead for a collision
        // (i.e., the length of the collision ray)
        float raysLength = 7f;
        // Calculate the collision ray vector
        Vector3 forwardRay = agent.velocity;
        forwardRay.y = 0f;
        forwardRay.Normalize();
        // shoot the ray from the character's current position
        Vector3 rayStartPos = agent.position;
        // holds the collision position and normal if collision detected

        // Find the collision

        if (Physics.SphereCast(rayStartPos, 0.7f, forwardRay, out hit, raysLength)) {

            Debug.DrawRay(rayStartPos, hit.point, Color.green);

            if (target == null || hit.transform != target.transform) {
                Vector3 newTarget = -hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
                dir.Normalize();
                dir *= maxAcceleration;
                return dir;
  
            }
        }
        // Update the wander orientation
        wanderOrientation += (Random.value - Random.value) * wanderRate;
        // Calculate the combined target orientation
        float target_orientation = agent.orientation + wanderOrientation;
        // Calculate the center of the wander circle
        Vector3 target_position = agent.position + OrientationVector(agent.orientation) * wanderOffset;
        wanderCircleCenter = target_position;
        // Calculate the target location
        target_position += OrientationVector(target_orientation) * wanderRadius;
        // Delegate to face
        Vector3 steering = target_position - agent.position;
        /*
        NPCController newTarg = new NPCController();
        if (target == null) {
            target = newTarg;
        }
        */
       // target.orientation = Mathf.Atan2(-steering.x, steering.z);
        // Now return the linear acceleration to be at full
        // acceleration in the direction of the orientation
        return steering.normalized * maxAcceleration;
    }
    public Vector3 CollisionPrediction() {

        Vector3 distance = target.position - agent.position;
        Vector3 veloDiff = target.velocity - agent.velocity;
        float t_closest = -Vector3.Dot(distance, veloDiff) / Mathf.Pow(veloDiff.magnitude, 2f);
        if(t_closest < 0) {
            return Vector3.zero;
        }
        Vector3 agentFuture = agent.position + agent.velocity * t_closest;
        Vector3 targetFuture = target.position + target.velocity * t_closest;
        if((agentFuture - targetFuture).magnitude < 1f) {
            Vector3 evasion = -agent.velocity;
            agent.DrawCircle(evasion.normalized * maxAcceleration, 0.4f);
            return evasion.normalized * maxAcceleration;
        }
        else {
            return Vector3.zero;
        }
    }
    // return a structure 

    public Vector3 WallAvoidance() {

        // holds a collision detector 
        RaycastHit hit;
        // Holds the minimum distance to a wall (i.e., how far
        // to avoid collision) should be greater than the
        // radius of the character.
        float avoidDistance = 8f;
        // Holds the distance to look ahead for a collision
        // (i.e., the length of the collision ray)
        float raysLength = 3f;
        float whiskerLength = 0.7f;
        // Calculate the collision ray vector
        Vector3 forwardRay = agent.velocity;
        forwardRay.y = 0f;
        forwardRay.Normalize();
        float veloDir = Mathf.Atan2(agent.velocity.x, agent.velocity.z);
        float leftWhiskerDir = veloDir + Mathf.PI / 4;
        Vector3 leftWhisker = new Vector3(Mathf.Cos(leftWhiskerDir), 0f, Mathf.Sin(leftWhiskerDir));
        leftWhisker.Normalize();
        float rightWhiskerDir = veloDir - Mathf.PI / 4;
        Vector3 rightWhisker = new Vector3(Mathf.Cos(rightWhiskerDir), 0f, Mathf.Sin(rightWhiskerDir));
        rightWhisker.Normalize();
        // holds the zero vector for no collision
        Vector3 noCollision = new Vector3(0f, 0f, 0f);

        // shoot the ray from the character's current position
        Vector3 rayStartPos = agent.position;
        // holds the collision position and normal if collision detected
  
        // Find the collision
        
        if (Physics.SphereCast(rayStartPos, 0.7f, forwardRay, out hit, raysLength)) {

            Debug.DrawRay(rayStartPos, hit.point,Color.green);

            if (target == null || hit.transform != target.transform) {
                Vector3 newTarget = -hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
                dir.Normalize();
                //dir += hit.normal * avoidDistance;
                dir *= maxAcceleration;
                return dir;
            }
        }
        /*
        if(Physics.SphereCast(rayStartPos, 0.4f, leftWhisker, out hit, whiskerLength)){
            Debug.DrawRay(rayStartPos, hit.point, Color.blue);
            if (target == null || hit.transform != target.transform) {
                collisionPosition = hit.transform.position;
                collisionNormal = hit.normal;
                Vector3 newTarget = hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
                //Vector3 dir = collisionPosition - agent.position;
                dir.Normalize();
               // dir += hit.normal * avoidDistance;
                dir *= maxAcceleration;
                return dir;
            }
        }
        */
        /*
        if (Physics.SphereCast(rayStartPos, 0.4f, rightWhisker, out hit, whiskerLength)){
            Debug.DrawRay(rayStartPos, hit.point, Color.red);
            if (target == null || hit.transform != target.transform) {
                collisionPosition = hit.transform.position;
                collisionNormal = hit.normal;
                Vector3 newTarget = hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
                //Vector3 dir = collisionPosition - agent.position;
                dir.Normalize();
              // dir += hit.normal * avoidDistance;
                dir *= maxAcceleration;
                return dir;
            }
           
        } 
        */
           
        return noCollision;
        
    }
    

    public float LookWhereYoureGoing() {

        if (agent.velocity == Vector3.zero) {
            return 0f;
        }
        // Create the structure to hold our output
        float steering_angular;
        float direction = Mathf.Atan2(-agent.velocity.x, agent.velocity.z);
        if(Mathf.Abs(direction) == 0) {
            return 0;
        }
        // Get the naive direction to the target
        float rotation = direction - agent.orientation;
      //  Debug.Log("this is the direction" + direction);
        // map the result to the (-pi,pi) interval 
        while (rotation > Mathf.PI) {
            rotation -= 2 * Mathf.PI;
        }
        while (rotation < -Mathf.PI) {
            rotation += 2 * Mathf.PI;
        }
        float rotationSize = Mathf.Abs(rotation);
       // Debug.Log(rotationSize);
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

    public Vector3 followPath() {
        // holds collision detector for following path
        RaycastHit hit;
        // Holds the minimum distance to a wall (i.e., how far
        // to avoid collision) should be greater than the
        // radius of the character.
        float avoidDistance = 15f;
        // Holds the distance to look ahead for a collision
        // (i.e., the length of the collision ray)
        float raysLength = 3f;
        // Calculate the collision ray vector
        Vector3 forwardRay = agent.velocity;
        //forwardRay.y = 0f;
        forwardRay.Normalize();
        // shoot the ray from the character's current position
        Vector3 rayStartPos = agent.position;
        // holds the collision position and normal if collision detected

        // Find the collision

        if (Physics.SphereCast(rayStartPos, 0.7f, forwardRay, out hit, raysLength)) {

            Debug.DrawRay(rayStartPos, hit.point, Color.green);

            if (target == null || hit.transform != target.transform) {
                Vector3 newTarget = -hit.transform.position + hit.normal * avoidDistance;
                Vector3 dir = newTarget - agent.position;
               // agent.DrawCircle(dir, 0.4f);
                dir.Normalize();
                dir *= maxAcceleration;
                return dir;

            }
        }
        Vector3 pathTarget = Path[current].transform.position;
       
        while (Vector3.Distance(agent.position, pathTarget) < 1f) {
            current++;
            if(current > Path.Length) {
                return Vector3.zero;
            }
            pathTarget = Path[current].transform.position;
        }
        Vector3 direction = pathTarget - agent.position;
        //agent.DrawCircle(direction, 0.4f);
        return direction.normalized * maxAcceleration;
    }

    public Vector3 stop() {
        Vector3 direction = -agent.velocity;
        return direction.normalized * maxAcceleration;
    }

}

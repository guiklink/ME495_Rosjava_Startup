/*
 * Copyright (C) 2014 Klink.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava_catkin_package_a.my_pub_sub_tutorial;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;  // This library give us the AbstractNodeMain interface (see ahead)
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;          
import org.ros.node.topic.Publisher;  // Import the publisher
import geometry_msgs.Twist;           // Import geometry_msgs.Twist ... remember to incluse this message into your dependencie files (see mini-project_rosjava.md)

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class Talker extends AbstractNodeMain { // Java nodes NEEDS to implement AbstractNodeMain

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/talker");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final Publisher<geometry_msgs.Twist> publisher =
        connectedNode.newPublisher("/turtle1/cmd_vel", geometry_msgs.Twist._TYPE); // That's how you create a publisher in Java!
    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        geometry_msgs.Twist twist = publisher.newMessage(); // Init a msg variable that of the publisher type
        sequenceNumber++;

        if (sequenceNumber % 3 == 0) {          // Every 3 executions of the loop (aprox. 3*1000ms = 3 sec)
          twist.getAngular().setZ(Math.PI/2);   // Steer the turtle left
        }
        else{
          twist.getLinear().setX(2);            // In the meantime keeps going foward 
        }
        
        publisher.publish(twist);       // Publish the message (if running use rostopic list to see the message)

        Thread.sleep(1000);             // Sleep for 1000 ms = 1 sec
      }
    });
  }
}

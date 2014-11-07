Sending Messages for the Turtle
===============================


## Overview ##

In this section we are going to walk though on how to change the **Writing a Simple Publisher and Subscriber** subproject in order to publish messages for the [turtlesim](http://wiki.ros.org/turtlesim). Also, you can `git clone` the final package from [here](https://github.com/guiklink/ME495_Rosjava_Startup).

*NOTE:* Create a [Simple Publisher and Subscriber](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/WritingPublisherSubscriber%28Java%29) subproject if you still have not yet.

##Adding Dependencies##

First, we need to add the dependencies we are going to be using for this task. Remember that the [turtlesim](http://wiki.ros.org/turtlesim) subscribes to 'geometry_msgs.Twist' ROS message, therefore we must make sure that the compiler is aware of this to build our executables.
 
Go inside your **rosjava** package (**rosjava_catkin_package_a** if you strickly followed the tutorial above) and open the [CmakeList.txt](https://github.com/guiklink/ME495_Rosjava_Startup/blob/master/CMakeLists.txt) and [package.xml](https://github.com/guiklink/ME495_Rosjava_Startup/blob/master/package.xml).

###CmakeList.txt###

```
##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rosjava_catkin_package_a)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools)
find_package(catkin REQUIRED COMPONENTS geometry_msgs)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)

catkin_package()

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})
```
The code line  ```find_package(catkin REQUIRED COMPONENTS geometry_msgs)``` must be added to include ***geometry_msgs***.


###package.xml###
~~~xml
<?xml version="1.0"?>
<package>
  <name>rosjava_catkin_package_a</name>
  <version>0.1.0</version>
  <description>The rosjava_catkin_package_a package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="klink@todo.todo">klink</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>Apache 2.0</license>


  <!-- Url tags are optional, but mutiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/rosjava_catkin_package_a</url> -->


  <!-- Author tags are optional, mutiple are allowed, one per tag -->
  <!-- Authors do not have to be maintianers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rosjava_build_tools</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <run_depend>geometry_msgs</run_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- You can specify that this package is a metapackage here: -->
    <!-- <metapackage/> -->

    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
~~~

The lines ```<build_depend>geometry_msgs</build_depend>``` and ```<run_depend>geometry_msgs</run_depend>``` to add **geometry_msgs**.


###Talker.java###

~~~java
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
~~~

~~~java
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;  // This library give us the AbstractNodeMain interface (see ahead)
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;          
import org.ros.node.topic.Publisher;  // Import the publisher
import geometry_msgs.Twist;           // Import geometry_msgs.Twist ... remember to incluse this message into your dependencie files 
~~~

Import a bunch of ROSjava classes that will be used in our code.   
[CancellableLoop](http://docs.rosjava.googlecode.com/hg/rosjava_core/html/javadoc/org/ros/concurrent/CancellableLoop.html) this is a loop that can be used similarly to the python command ```` while not rospy.is_shutdown():```.   
[AbstractNodeMain](http://docs.rosjava.googlecode.com/hg/rosjava_core/html/javadoc/org/ros/node/AbstractNodeMain.html) every node made in ROSJava must extend this abstract class in order to be viewed as a node.   
[ConnectedNode](http://docs.rosjava.googlecode.com/hg/rosjava_core/html/javadoc/org/ros/node/ConnectedNode.html) class for connected nodes (use explained ahead).   
[topic.Publisher](http://docs.rosjava.googlecode.com/hg/rosjava_core/html/javadoc/org/ros/node/topic/Publisher.html) makes the publish interface available to implement.    
[geometry_msgs.Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) import Twist messages.

~~~java
  public void onStart(final ConnectedNode connectedNode) {
    final Publisher<geometry_msgs.Twist> publisher =
        connectedNode.newPublisher("/turtle1/cmd_vel", geometry_msgs.Twist._TYPE); // That's how you create a publisher in Java!
~~~

On running time ```ConnectedNode connectedNode``` gives the connection between your node and the ```roscore`` **master** running.
Afterwards, a **publisher** is created from the connection, where the topic and message type is defined respectively.

~~~java
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;
      }
~~~

On the first line an **CancellableLoop** class is passed to be executed in our executing node. And initialize a ```sequenceNumber``` variable to keep track of the times that the loop is exectued.

~~~java
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
~~~

**CancellableLoop** must ```have a loop()``` method, which is obviously the loop that will be executed until the node is killed or stopped. ''' if (sequenceNumber % 3 == 0)''' at every 3 executions of the loop a twist message with a rotation on the Z-axis of 90 degrees is created to make the robot steer to the left. In every other execution a forward velocity twist message is created.
Finally, ```publisher.publish(twist);``` published the message and the node sleeps for 1000 ms.

##Compiling the Code##
There are two ways to compile your code:
1. Go to your workspace root directory and do a ```catkin_make```
2. Sometimes doing a ```catkin_make``` can take ages! In this cases you can have **gradle** to compile your local package. In order to do this go to your sub project directory and execute ```../gradlew installApp```. Again, check the Writing a [Simple Publisher and Subscriber](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/WritingPublisherSubscriber%28Java%29) tutorial for more information.

##You're ready to execute it...##
Execute a master ```roscore```, pop up the turtlesim ```rosrun turtlesim turtlesim_node``` and execute your brand new Java node to see your turtle make squares! 


### ME495 Mini-Project: rosjava
----------------------------

The goal is to install rosjava and modify the simple publisher to control the [turtlesim](http://wiki.ros.org/turtlesim) package.

## Installing rosjava

Install rosjava following the [source install instructions](http://wiki.ros.org/rosjava/Tutorials/indigo/Installation)

### rosjava_build_tools Tutorials

#### Tools Used

We use Gradle and Maven for the build and dependency management respectively.

##### Gradle

[Gradle](http://www.gradle.org/) is build automation tool. Gradle can automate the building, testing, publishing, deployment and more of software packages or other types of projects such as generated static websites, generated documentation or indeed anything else.Gradle uses [Maven](http://maven.apache.org/) for dependency management.

When you call catkin_make in rosjava, compilation task is handed over to gradle instead of cMake. 

#### Creating Rosjava Packages

Create a [rosjava package](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/Creating%20Rosjava%20Packages).Follow the directions upto step 5.1 and then continue to the next step 'Writing a Simple Publisher and Subscriber (Java)'. [Note: Do not execute Step 5.2. It seems to break catkin_make and compilation error follows anything you do in rosjava after that].

##### Repository Structure

When you run catkin_create_rosjava_pkg, rosjava create the workspace a little bit different than the ros catkin_workspace. 


**Ros** | **Rosjava**
------------ | -------------
Catkin Stack | Catkin Package/Gradle Multi-project
Catkin Package | Gradle Subproject

The typical structure of a rosjava workspace looks like this:

```javascript
+ rosjava_foo
 - package.xml
 - CMakeLists.txt
 - build.gradle
 - settings.gradle
 - gradlew
 + projectA
   - build.gradle
   + src/main/java
   + src/main/tests 
 + projectB
   - build.gradle
   + src/main/java
   + src/main/tests 
   ```



## Writing a Simple Publisher and Subscriber (Java)

Create a simple publisher and suscriber following the tutorial [Writing a Simple Publisher and Subscriber](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/WritingPublisherSubscriber%28Java%29). Check to make sure the build.gradle file has the dependency listed. 

### Changes to control the turle

Create a new publisher that publishes to rostopic /turtle1/cmd_vel

```javascript
connectedNode.newPublisher("/turtle1/cmd_vel", geometry_msgs.Twist._TYPE);
```
create a new geometry message

```javascript
geometry_msgs.Twist twist = publisher.newMessage();
```
publish the twist message you created

```javascript
 publisher.publish(twist);
 ```
update cMakeLists.txt

```javascript
find_package(catkin REQUIRED rosjava_build_tools)
find_package(catkin REQUIRED COMPONENTS geometry_msgs)
```

## Android Installation

Follow the instruction for [android installation](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/Creating%20Android%20Packages). Whe you get to the [Installation - ROS Development Environment](http://wiki.ros.org/android/Tutorials/hydro/Installation%20-%20Ros%20Development%20Environment), follow these steps before installation

```javascript
source ~/rosjava/devel/setup.bash
```

set up ANDROID_HOME. eg:

```javascript
export ANDROID_HOME="/opt/android-studio/sdk"
```


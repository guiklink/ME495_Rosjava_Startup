
# ME495 Mini-Project: rosjava
----------------------------

The goal of our project is to install rosjava and modify the simple publisher to control the [turtlesim](http://wiki.ros.org/turtlesim) package. The extensions are a demo of Android application publishing "Hello World" message from Moto G and subscribing on our computer, and reading accelerometer readings from the phone and sending the phone’s orientation to the computer.

## Installing rosjava

Install rosjava following the [source install instructions](http://wiki.ros.org/rosjava/Tutorials/indigo/Installation). One of the problems we came across was catkin_make failing during the installlation. Updating all the packages seems to have solve the problem. There is no explanation why this is happening. On cautionary side, run a 'sudo apt-get update' before rosjava installation. [Damon Kohler](http://www.damonkohler.com/) is the only person seems to know the inner workings of rosjava since it was one of his projects. 

## Writing a Simple Publisher and Subscriber (Java)

A typical rosjava workspace looks slightly different than a ROS workspace. rosjava is not as tightly integrated with ROS as rospy or roscpp. rosjava uses Gradle for build.

#### Gradle and Maven

[Gradle](http://www.gradle.org/) is a project automation tool. Gradle can automate the building, testing, publishing, deployment and more of software packages or other types of projects such as generated static websites, generated documentation. The Gradle Wrapper is another word you might come across. It is the preferred way of starting a Gradle build. The wrapper is a batch script on Windows, and a shell script for other operating systems. When you start a Gradle build via the wrapper, Gradle will be automatically downloaded and used to run the build. Gradle's build scripts are written in Groovy, not XML. [Groovy](http://groovy.codehaus.org/) is a dynamic language for the Java Virtual Machine and is inspired by languages like Python and Ruby. 

When you call catkin_make in rosjava, compilation task is relayed over to gradle from cMake. As far as we are concerned, we might have to add dependencies in build.gradle file. A typical build.gradle file for a project looks like:

```javascript
/*
 * Copyright (C) 2014 Remi Padiath.
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

/* This plugin is necessary for creating installApp tasks (i.e. executables) */
apply plugin: 'application'
mainClassName = 'org.ros.RosRun'

/* 
 Dependencies can be on external maven artifacts (such as rosjava_core
 here) or on sibling subprojects. Fpr external maven artifact dependencies
 it's convenient to use an open ranged dependency, but restrict it to
 cover the patch version only to avoid breakages due to api changes
 which usually occur on minor and major version numbers.
*/

dependencies {
  /* An external maven artifact dependency */
  compile 'org.ros.rosjava_core:rosjava:[0.1,)'
  compile 'org.ros.rosjava_messages:geometry_msgs:1.10.+'
  /* Example of a local subproject dependency */ 
  /* compile project(':sibling_gradle_project') */
}
```

[Maven](http://maven.apache.org/) is a build automation tool used primarily for Java projects. Maven addresses two aspects of building software: First, it describes how software is built, and second, it describes its dependencies. It also forces a lot of best practices and standards. Most of the java community uses Maven repository structure and standards as well as the dependency management. Gradle has a Maven plugin that adds support for deploying to Maven repositories. 


#### Creating Rosjava Packages

First, create a wokspace for your project. Then, create a [rosjava package](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/Creating%20Rosjava%20Packages). Follow the directions upto step 5.1.Binary Projects (App) and then continue to the next step [Writing a Simple Publisher and Subscriber (Java)](http://wiki.ros.org/rosjava_build_tools/Tutorials/hydro/WritingPublisherSubscriber%28Java%29). [Note: Do not execute Step 5.2.Library Project. It seems to break catkin_make and compilation error follows anything you do in rosjava after that].

##### Repository Structure

When you run catkin_create_rosjava_pkg, rosjava create the repository slightly different to usual ros repository structures.


**Ros** | **Rosjava**
------------ | -------------
Catkin Stack | Catkin Package/Gradle Multi-project
Catkin Package | Gradle Subproject

The typical structure of a rosjava repository looks like this:

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
As you can see, there is build.gradle file for each projects or subprojects. And it is possible to just compile the packages using gradle instead of using catkin_make after you have run catkin_make at least once. 

A typical package.xml looks as follows. 

```javascript
<?xml version="1.0"?>
<package>
  <name>rosjava_catkin_package_a</name>
  <version>0.1.0</version>
  <description>The rosjava_catkin_package_a package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="remi@todo.todo">remi</maintainer>


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
  <build_depend>rosjava_messages</build_depend>
  <run_depend>rosjava_messages</run_depend>

 <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- You can specify that this package is a metapackage here: -->
    <!-- <metapackage/> -->

    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```
And a typical CMakeLists.txt is shown here:

```javascript
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

##### Compiling with Gradle

When you run catkin_make, cMake runs through your entire workspace and when it gets to your new project, it will pass off the build to gradle.You could alternatively just compile your subproject alone with gradle (much faster than running catkin_make across your entire workspace):

```javascrpit
source devel/setup.bash
cd src/rosjava_catkin_package_a/my_pub_sub_tutorial
../gradlew installApp
```

Next, [move the turtle](https://github.com/guiklink/ME495_Rosjava_Startup/blob/master/howToUseJavaControlTurtle.md) or [create an Android workspace](https://github.com/guiklink/ME495_Rosjava_Startup/blob/master/creatingAndroidEnv.md).


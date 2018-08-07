# ROS Anomaly Detector package


# Overview

The ROS Anomaly Detector Module (ADM) is designed to execute alongside
industrial robotic arm tasks to detect unintended deviations at the application
level. The ADM utilizes a learning based technique to achieve this. The process
has been made efficient by building the ADM as a ROS package that aptly fits in
the ROS ecosystem. While this module is specific to anomaly detection for an
industrial arm, it is extensible to other projects with similar goals. This
is meant to be a starting point for other such projects.
<!-- Figure \ref{fig:ros_i_overview} presents the ROS-I
architecture. The red box around ROS ADM in the ROS Layer shows where the ADM
exists within the ROS ecosystem. -->

The crux of anomaly detection within this module relies on a three step
process. The steps includes creating datasets out of the published messages
within ROS, training learning models from those datasets, and deploying it in
production. Appropriately, the three modes that the ROS ADM can be executed in
are Collect, Learn, and Operate. The following image presents the high level
workflow that is meant to be followed.

![Image](images/adm_flow.png)

<!-- To evaluate our anomaly detection methodology we considered an industrial arm
programmed and controlled with ROS. We developed a ROS Anomaly Detection Module
(ADM) that is designed to execute alongside manufacturing tasks to detect
unintended arm deviations. ADM can be executed in three different modes for
collecting data (Collect mode), learning models (Learn mode), and detecting
anomalies (Operate mode). -->



<!-- % % Also important, the IDM is designed to integrate well
% % with the development of any ROS application.
% The IDM addresses the integrity of the industrial arm task by
% monitoring the captured joint states that are relayed back to the
% control system from the sensors. At any given point, when the joints
% have progressed into an unseen state, they should be marked anomalous.
-->



What are the different modes?
What happens when a mode is initialized?

### Collect Mode
The ADM is first run in the Collect mode after the task has been programmed and
assessed in the commissioning phase of the robot. This mode runs AND collects
data as a node by subscribing to the '/joint\_states' topic, of message type
'sensor\_msgs/JointState,' within ROS.

### Learn Mode
The goal of this mode is submitting the user with trained classifiers. In
order to accomplish this the user is depended on to supply not only the datasets
for training, but also the classifiers of interest.
These are supplied to an internal library via a script. The internal library semi-automates the learning process. Particularly, the user is only required to supply their script to the Interface component. It in turn, pre-processes the
data (dimensionality reduction if specified), and the set of classifiers are
all trained with their appropriate datasets. After training, the classifiers are
saved in storage.

#### Learning library

- Interface
- Data Process
- Data Persist
- Anomaly Detector
- Output Matrix

<!-- - Listening mode - Collect data, specify topics.
- Training option - Use collected data and train a model.
	Needs lots of options and cover various attributes.
	How many different models?
	How much data?
	Use incoming data for training?
	The ML techniques details are here

	Split unlabeled data collection, and labeled data collection
		process to make sure correct labels are presented.

- Operational mode - Import trained model and run task. -->

### Operate Mode
In the Operate mode, the
classifier that was determined the best is put into effect as the robot is
deployed to execute its task, along with the other saved preprocessing models.
It is worth noting that the task the robot executes is the same one that was
initially programmed in the commissioning phase. The Operate mode runs as a
node, so it subscribes to the '/joint\_states' topic. The data is preprocessed
and fed to the determined classifier for a prediction. The user is warned of
anomalous data points, and is encourages to take action when the anomaly score
surpasses the threshold.


---

## Dependencies

[ROS](http://wiki.ros.org/ROS/Installation) needs to be first installed

catkin build
ros related items needed

ros-core


moveit
	- ros_moveit_core
	- moveit_setup_assistant

ros industrial core
motoman driver

### Dependencies independet of ROS

Sci-Kit Learn - [License](https://github.com/scikit-learn/scikit-learn/blob/master/COPYING)
Numpy - [License](https://docs.scipy.org/doc/numpy-1.10.0/license.html)
Pandas - [License](https://pandas.pydata.org/pandas-docs/stable/overview.html#license)

---

## Build

setup.py should run from home directory
specify the 'learn_lib' package to build within the src/ directory

then go back and build the whole ROS workspace
Then scripts should be able to run


 -- Build entire workspace!!!!!

```bash
	python setup.py build
```
---

<!-- ## Install

```bash
	python setup.py install
```

--- -->

## Usage

How is it included in roslaunch files?

How is it run?

```bash
	roslaunch <pkg_name> <file>.launch
```

---

<!-- ## Demo

- Demo includes moveit_setup_assistant package, data collected,
	and the trained classifiers
- User should be able to use it out of the box for testing.

--- -->

## Troubleshooting

What are the gotchas?

1. Make sure to always supply full paths, instead of relative paths

---

## License

BSD-3

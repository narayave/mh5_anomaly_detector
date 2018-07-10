# ROS Anomaly Detector package


# Overview

What does this module do? How is it used?
What are the different modes?
What happens when a mode is initialized?


- Listening mode - Collect data, specify topics.
- Training option - Use collected data and train a model.
	Needs lots of options and cover various attributes.
	How many different models?
	How much data?
	Use incoming data for training?
	The ML techniques details are here

	Split unlabeled data collection, and labeled data collection
		process to make sure correct labels are presented.

- Operational mode - Import trained model and run task.

---

## Dependencies

catkin build
ros related items needed

---

## Build

```bash
	python setup.py build
```
---

## Install

```bash
	python setup.py install
```

---

## Usage

How is it included in roslaunch files?

How is it run?

```bash
	roslaunch <pkg_name> <file>.launch
```

---

## Demo

- Demo includes moveit_setup_assistant package, data collected,
	and the trained classifiers
- User should be able to use it out of the box for testing.

---

## Troubleshooting

What are the gotchas?

---

## License

BSD

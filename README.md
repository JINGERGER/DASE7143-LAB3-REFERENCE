# DASE7143 Lab 3 Manual: Target Search and Approach

This manual provides a practical workflow for building, testing, and evaluating a ROS 2 robot that:

1. searches for the required trash bin, which starts outside the camera field of view;
2. aligns the object with the camera centre;
3. approaches it safely; and
4. stops with a front clearance **greater than 0.40 m**.

Rotation in place may be used during the initial search. Supporting other target objects is an extension and should be implemented through configuration rather than by duplicating control code.

> [!IMPORTANT]
> This repository is an engineering guide, not an executable ROS 2 package or an official replacement for the course handout. It assumes the Bunker Mini sensor and VLM setup completed in the prerequisite lab. Confirm the official lab title and submission format with the teaching team.

## Learning outcomes

After completing the lab, you should be able to:

- connect LiDAR, RGB, depth, inference-result, and velocity-command interfaces in ROS 2;
- associate a detected target with a valid depth measurement;
- reject malformed, stale, or out-of-order perception results;
- implement search, alignment, approach, and stopping as a state machine;
- tune motion safely using measurements from the actual robot; and
- evaluate performance with repeatable trials and quantitative evidence.

## Before you start

### Required resources

- The Bunker Mini and Orin used in the prerequisite lab.
- The 32-line LiDAR and Intel RealSense D555 RGB/depth camera.
- The configured `simple_vlm` package and group DashScope API access.
- The ROS 2 workspace and Bunker, LiDAR, camera, and OctoMap packages used previously.
- A trash bin for the required task and any additional target objects used for extensions.
- A cleared test area, tape measure, and accessible emergency stop.
- Permission to record video or ROS bag data, if required for the report.

### Required prerequisite-lab completion

The prerequisite lab contains **four Checkpoints**. Complete them, obtain TA verification, include the requested evidence in the presentation video, and answer its Questions in the report:

| Prerequisite checkpoint | Required result |
| --- | --- |
| LiDAR and OctoMap | Visualize the LiDAR point cloud and the octree generated on `/octomap_full` |
| Stereo camera | Visualize live RGB and depth images in RViz and measure their rates and bandwidth |
| Chassis motion | Configure the detected CAN interface, launch `bunker_base`, and verify rotation through `/cmd_vel` |
| VLM with ROS 2 | Run `simple_vlm` and observe image-analysis output on `/vlm_result` |

The prerequisite lab establishes these default interfaces:

| Data | Default topic |
| --- | --- |
| LiDAR point cloud | `/cx/lslidar_point_cloud` |
| RGB image | `/camera/camera/color/image_raw` |
| Depth image | `/camera/camera/depth/image_rect_raw` |
| VLM result | `/vlm_result` |
| Chassis command | `/cmd_vel` |

Treat these as known starting points, not immutable values. Use `ros2 topic list -t` on the current robot to verify that the running configuration still matches them. The CAN interface is machine-dependent; use the value reported by `find_all_can_port.sh` rather than copying `can2` blindly.

### Safety rules

- Perform the first tests with the drive wheels raised or motion disabled.
- Keep linear and angular speeds low until their signs and units are verified.
- Keep a person beside the emergency stop during every hardware trial.
- Use a cleared, supervised test area; this exercise is not a complete collision-avoidance system.
- Stop immediately when target, depth, or obstacle information becomes invalid.

## Lab workflow

Complete the milestones in order. Do not start autonomous approach testing until the preceding checkpoint passes.

| Milestone | Task | Checkpoint |
| --- | --- | --- |
| 1 | Inspect ROS interfaces | Correct topics, message types, QoS, frames, and units are recorded |
| 2 | Validate LiDAR safety data | Fresh point clouds produce a footprint-aware protective region |
| 3 | Validate perception | The requested target is detected and malformed outputs are rejected |
| 4 | Validate target depth | Stationary distance estimates agree with manual measurements |
| 5 | Implement search and alignment | The robot finds and centres targets at low speed |
| 6 | Implement approach and stop | The robot stops inside the declared safe interval |
| 7 | Test failure handling | Stale LiDAR, stale VLM data, target loss, and invalid depth always produce a stop |
| 8 | Evaluate | Repeated trials, logs, measurements, and failure notes are saved |

## Quick completion checklist

- [ ] Define the target and a stopping interval entirely above 0.40 m.
- [ ] Record topic names, message types, QoS, frames, and depth units.
- [ ] Define and test a LiDAR-based protective stop region around the robot path.
- [ ] Validate structured perception output and source timestamps.
- [ ] Measure depth only inside the target region.
- [ ] Implement `SEARCH`, `ALIGN`, `APPROACH`, `VERIFY_STOP`, `DONE`, and `HOLD`.
- [ ] Add freshness limits, motion timeouts, speed limits, and a command watchdog.
- [ ] Test each state independently before running the complete sequence.
- [ ] Run repeated trials and measure final clearance from the robot front.
- [ ] Save configuration, code revision, logs, videos, and report evidence.

## Why the supplied scaffold needs improvement

The supplied reference is intentionally incomplete: missing subscriptions, parameters, and control logic are student exercises. The more important engineering issues are the assumptions connecting perception to motion.

| Reference behavior or assumption | Why it matters | Proposed improvement |
| --- | --- | --- |
| Measure the central 20 × 20 depth patch | The patch may contain a wall, floor, or another object rather than the target | Associate depth with the detected target region |
| Accept `trash_bin:left/right/middle` without an image timestamp | A delayed answer may describe a previous robot orientation | Attach the source image timestamp and reject obsolete observations |
| Match output using substring searches | Explanations, contradictory answers, or multiple objects can trigger unintended actions | Validate a strict output schema |
| React to detections only in search states | Alignment and approach need continued observation | Check target validity and alignment throughout motion |
| Compute turns from speed × duration | Actual rotation depends on drivetrain behavior and surface conditions | Use visual feedback and, when available, odometry |
| Keep the previous distance after a depth exception | Old measurements may continue to authorize motion | Invalidate failed measurements and enforce freshness limits |
| Assume depth is `16UC1` in millimeters | A different encoding or unit convention produces incorrect distances | Inspect the driver contract and explicitly normalize units |
| Use a single stopping threshold | Camera offset, latency, braking, and noise affect final clearance | Define the measurement reference and include a measured margin |

These are limitations of the supplied scaffold, not claims about every possible completion of its missing control loop.

## 1. Define success before tuning

Use **clearance from the robot's front boundary to the target surface** as the working distance definition, unless the teaching team specifies another reference.

Choose a desired stopping clearance and an acceptable interval before testing. The entire interval must be above 0.40 m. Record the chosen values in your report; do not treat a sensor reading of exactly 0.40 m as successful completion.

A successful trial should satisfy all of the following:

- The robot approaches the requested object rather than a distractor.
- The target is aligned within the configured image-space tolerance.
- The robot stops within the declared clearance interval.
- The robot remains stopped until a new task or explicit reset.
- There is no contact with the target or surrounding obstacles.

A lower bound alone is insufficient: stopping several meters away would satisfy `> 0.40 m` without demonstrating useful approach behavior.

## 2. Inspect the robot interfaces

Source the workspace in every new terminal before launching workspace packages:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Keep each long-running launch command open and use another terminal for inspection. Bring up only the components required for the current test:

```bash
# LiDAR
ros2 launch lslidar_driver lslidar_cx_launch.py

# RealSense D555
ros2 launch realsense2_camera rs_launch.py

# Bunker Mini base; replace <can_port> with the detected interface
ros2 launch bunker_base bunker_base.launch.py port_name:=<can_port>

# VLM; use the existing group configuration
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
ros2 run simple_vlm simple_vlm_node --ros-args \
  -p config_file:=src/simple_vlm/config.yaml
```

Do not place an API key in source code, screenshots, logs, or this repository.

Before implementing subscriptions, inspect the actual running system:

```bash
ros2 topic list -t
ros2 topic info --verbose /camera/camera/color/image_raw
ros2 topic info --verbose /camera/camera/depth/image_rect_raw
ros2 topic info --verbose /vlm_result
ros2 topic info --verbose /cmd_vel
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /vlm_result
ros2 interface show <message_type>
```

Replace any default topic that differs on the running system. Confirm:

- ROS 2 distribution and installed camera driver.
- RGB and depth message types, encodings, frame IDs, and timestamps.
- Whether depth is registered to the RGB optical frame and pixel grid.
- Camera calibration and any resizing, cropping, or letterboxing applied before inference.
- VLM request/output format and typical end-to-end latency.
- Velocity command type, topic, coordinate convention, and command timeout.
- Publisher/subscriber QoS compatibility.
- Camera position relative to the robot's front boundary.

The prerequisite lab controls the Bunker Mini using `geometry_msgs/msg/Twist` on `/cmd_vel`; verify this before use. Ensure only the intended controller owns motion commands, or use the platform's command arbitration mechanism.

For camera subscriptions, select QoS compatible with the actual publisher. A Reliable subscriber cannot receive from a Best Effort publisher. See the [ROS 2 QoS documentation](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html); use the documentation matching your installed distribution.

Record the interface contract before writing control logic:

| Interface | Topic | Message type | QoS | Frame/units | Expected rate |
| --- | --- | --- | --- | --- | --- |
| RGB image | `/camera/camera/color/image_raw` | verify with `ros2 topic list -t` | `<fill in>` | `<fill in>` | `<measure>` |
| Depth image | `/camera/camera/depth/image_rect_raw` | verify with `ros2 topic list -t` | `<fill in>` | `<fill in>` | `<measure>` |
| Inference result | `/vlm_result` | verify with `ros2 topic list -t` | `<fill in>` | text plus source metadata if extended | `<measure>` |
| LiDAR point cloud | `/cx/lslidar_point_cloud` | verify with `ros2 topic list -t` | `<fill in>` | `<fill in>` | `<measure>` |
| Velocity command | `/cmd_vel` | `geometry_msgs/msg/Twist` | `<fill in>` | m/s and rad/s | controller-dependent |

**Checkpoint:** while the robot is stationary, each required subscription receives data at the expected rate and publishing an explicit zero command keeps the robot stopped.

## 3. Make perception outputs unambiguous

Use a configurable target description rather than hard-coding `trash_bin` into every callback.

For a baseline system with directional output, a prompt can be:

```text
Inspect this image for the requested target: {target_description}.
Return exactly one JSON object, with no Markdown or explanation.

If a matching object is clearly visible:
{"found": true, "direction": "left"}

The only allowed directions are "left", "middle", and "right".
Use the center of the object's visible extent to choose the horizontal
third of the image. If several objects match, select the one whose center
is closest to the image center.

If no matching object is visible, or you cannot decide:
{"found": false, "direction": null}

Do not infer that an object exists outside the image.
```

This is a baseline protocol, not a guarantee of reliable recognition. Validate field types and allowed values in code; malformed or contradictory output must not authorize forward motion.

**Directional output alone is insufficient for reliable target-specific depth sampling.** For the improved system, obtain a bounding box or mask using a localization-capable model or a detector/tracker. Check localization quality rather than assuming any VLM can produce accurate boxes.

The perception adapter should produce an observation containing:

| Field | Purpose |
| --- | --- |
| `request_id` | Associate output with a specific inference request |
| `source_image_stamp` | Identify when the observed image was captured |
| `target_label` | Identify the requested target category |
| `found` | Explicitly represent target absence |
| `bbox_xyxy` or mask | Locate the target in the original RGB image coordinates |
| `track_id`, if available | Maintain target identity between observations |

The adapter must copy request IDs and timestamps from the input pipeline. Do not ask the model to invent them. Undo inference-image resizing and padding before using coordinates on the original image.

Repeated confirmation can reduce transient false detections. It does not eliminate systematic misclassification, and model-reported confidence should not be treated as a calibrated probability.

**Checkpoint:** test visible, absent, ambiguous, and malformed cases while stationary. Only a valid, current observation of the requested target may transition the controller out of search.

## 4. Associate depth with the target

1. Pair the RGB observation with temporally compatible depth.
2. Use depth registered to the RGB image, or explicitly transform and project depth using camera calibration.
3. Select a target mask or an interior region of its bounding box.
4. Reject invalid, non-finite, and out-of-range samples according to the driver specification.
5. Require sufficient valid coverage and a plausible distribution of depths.
6. Estimate target depth robustly and record its timestamp and validity.

A median is useful only when enough selected pixels belong to the target. A narrow bottle or a loose bounding box can contain mostly background. A mask, spatially coherent depth cluster, or a better localized region may be necessary.

Read the source encoding without silently changing its physical interpretation. Standard ROS depth conventions use meters for `32FC1` and millimeters for the `16UC1` representation, but confirm what the actual camera publishes. See [REP 118: Depth Images](https://www.ros.org/reps/rep-0118.html).

On conversion failure, insufficient coverage, or a depth timeout, invalidate the distance. Never continue approaching using an indefinitely cached value.

Also distinguish optical-axis depth from robot clearance. For an aligned, forward-facing camera mounted `b` meters behind the front boundary, a simplified estimate is:

```text
front_clearance ≈ target_depth - b
```

This approximation requires suitable mounting geometry and target alignment. Use calibrated transforms and robot geometry when those assumptions do not hold.

**Checkpoint:** place the target at several measured distances and image positions. Record estimated depth, front-clearance estimate, manual clearance, valid-pixel ratio, and error. Resolve large or position-dependent errors before enabling forward motion.

## 5. Handle delayed inference explicitly

Running perception and control at different rates is normal. The problem is using an observation after the scene or robot pose has changed.

### Baseline: stop, observe, act

For an implementation that is straightforward to debug:

1. Rotate a small amount during search.
2. Stop and allow the camera view to settle.
3. Submit a newly captured frame for inference.
4. Accept only the result associated with that observation.
5. Perform a bounded adjustment or approach segment.
6. Stop and obtain another observation.

During every motion segment, continue checking fresh depth and the available obstacle monitor. An inference timeout should keep the robot stopped.

This approach trades speed for simpler timing. An observation from before the latest movement must not authorize the next movement.

### Improved: asynchronous perception and tracking

Once the baseline works:

- Keep the control timer independent of inference completion.
- Avoid blocking inference calls inside the control callback.
- Bound the inference queue; replace pending obsolete frames with newer frames.
- Reject out-of-order and over-age results using source timestamps.
- Use a faster tracker between semantic detections.
- Revalidate target identity periodically and after tracking loss.

Freshness alone is not enough during rapid rotation: even a recent image may describe a significantly different pose. Use pose-aware compensation, current-frame tracking, or stop before acting on the result.

## 6. Use an explicit control state machine

| State | Behavior | Exit condition |
| --- | --- | --- |
| `SEARCH` | Rotate in bounded steps or under continuous fresh perception | A target is confirmed, or search times out |
| `ALIGN` | Correct horizontal target error | Alignment is stable; otherwise stop if observation is invalid |
| `APPROACH` | Move forward with bounded speed and continued alignment checks | Stopping distance reached, alignment lost, or perception becomes invalid |
| `VERIFY_STOP` | Command zero velocity and verify distance and stability | Success is confirmed or a new observation is needed |
| `DONE` | Hold zero command | Explicit reset or new task |
| `HOLD` | Stop because required data or clearance is unavailable | Fresh valid observations permit replanning |

Keep sensing callbacks focused on updating validated observations. Centralize motion decisions in the control loop so that callbacks cannot independently issue conflicting actions.

Implement and test the states incrementally:

1. Make every state publish zero velocity.
2. Add bounded `SEARCH` rotation and its timeout.
3. Add `ALIGN` with zero linear velocity.
4. Add low-speed `APPROACH` with continuous validity checks.
5. Add `VERIFY_STOP`, then require an explicit reset to leave `DONE`.
6. Force each sensor-failure path and confirm it enters `HOLD`.

The following is control pseudocode, not an executable ROS node:

```text
if task_complete:
    command_zero()
elif required_sensors_invalid_or_stale() or obstacle_stop_required():
    command_zero()
    enter_hold()
elif state == SEARCH:
    perform_bounded_search_or_confirm_target()
elif target_observation_invalid_or_stale():
    command_zero()
    enter_hold()
elif state == ALIGN:
    correct_heading_or_begin_approach()
elif state == APPROACH:
    if stopping_condition_reached():
        command_zero()
        enter_verify_stop()
    elif alignment_outside_tolerance():
        command_zero()
        enter_align()
    else:
        approach_with_bounded_speed()
elif state == VERIFY_STOP:
    command_zero()
    verify_clearance_and_stability()
else:
    command_zero()
```

Search does not require an already detected target, but it still requires whatever sensors and clearances are necessary to rotate safely.

### Alignment

With target center `u`, image width `W`, and an image that has not been mirrored:

```text
horizontal_error = (u - W / 2) / (W / 2)
angular_velocity = clamp(-k_heading * horizontal_error, -omega_max, omega_max)
```

Verify the sign on the actual platform at low speed. Under the usual ROS body convention, positive yaw turns left, while image coordinates increase to the right.

Use an alignment deadband and separate entry/exit tolerances to avoid repeated left-right switching.

### Approach and stopping

Reduce forward speed as clearance approaches the chosen setpoint:

```text
forward_speed = clamp(k_distance * (clearance - desired_clearance), 0, v_max)
```

Stop within a declared tolerance; proportional speed control alone may approach the setpoint indefinitely. Do not impose a minimum forward speed that overrides the stopping condition.

Estimate the additional clearance needed for stopping:

```text
stopping_margin ≈ speed * effective_latency
                  + speed² / (2 * effective_deceleration)
                  + measurement_margin
```

This is a tuning model, not a certified guarantee. Measure latency and braking behavior on the actual robot, include camera offset, and choose a desired clearance comfortably above the required minimum.

Do not automatically reverse after overshooting unless rear clearance is monitored. Mark the trial unsuccessful and stop when recovery is not supported.

## 7. Use LiDAR for motion safety

The depth camera estimates distance to the selected target; it does not cover the robot's full path or footprint. Use the LiDAR stream as a separate motion-safety input.

At minimum:

1. transform or interpret LiDAR points in a robot-fixed frame;
2. filter points by valid range and height where appropriate;
3. define a protective region covering the robot footprint plus braking margin;
4. stop if an obstacle enters that region;
5. invalidate LiDAR safety status when the point cloud becomes stale; and
6. apply the check during both rotation and forward motion.

Do not classify the selected target as a generic obstacle that permanently prevents the required approach. The controller must distinguish the target stopping condition from unrelated route obstacles, for example by combining target localization, depth, and spatial gating. Validate this behavior at low speed with a person ready at the emergency stop.

Using `/octomap_full` for persistent mapping, frontier-based spatial exploration, or planning around obstacles is an optional extension. A direct point-cloud safety check may be simpler for the baseline, but it must account for the robot footprint and the LiDAR blind region.

**Checkpoint:** while commanding only low-speed test motion, introduce an unrelated obstacle into the protective region and verify that the robot stops. Also stop the LiDAR publisher and verify that stale data cannot authorize continued movement.

## 8. Add operational safeguards

- Stop motion immediately when required LiDAR, target, or depth validity is lost.
- Set maximum search duration, movement duration, and approach travel limits.
- Use the base controller's command watchdog so process failure cannot leave a persistent movement command.
- Publish zero velocity during normal shutdown; a final message alone is not a substitute for a watchdog.
- Keep a physical emergency stop accessible during hardware tests.
- Treat obstacle detection separately from target-distance estimation: one target depth patch cannot protect the robot's entire footprint.

Full obstacle avoidance and spatial exploration are optional extensions. A LiDAR-based protective stop and supervised, cleared test area remain required for safe baseline testing.

## 9. Tune in a useful order

1. Verify message delivery, coordinate conventions, depth units, and LiDAR freshness.
2. Verify the LiDAR protective stop at low speed.
3. Verify target localization and distance while the robot is stationary.
4. Test low-speed rotation and alignment.
5. Test short approaches with generous stopping clearance.
6. Test target loss, invalid depth, delayed inference, and LiDAR loss.
7. Repeat trials before increasing speed.

Expose target description, topic names, speed limits, alignment tolerances, observation timeouts, stopping clearance, and search limits as ROS parameters.

Parameter values are platform-specific. Explain how each value was measured or selected rather than presenting an untested universal configuration.

Use a configuration record such as:

| Parameter | Value | Unit | Selection evidence |
| --- | --- | --- | --- |
| Desired front clearance | `<fill in>` | m | Must include measured stopping margin |
| Clearance tolerance | `<fill in>` | m | Based on depth noise and trial repeatability |
| Maximum linear speed | `<fill in>` | m/s | Safe stopping tests |
| Maximum angular speed | `<fill in>` | rad/s | Low-speed alignment tests |
| Alignment entry/exit tolerance | `<fill in>` | normalized image error | Oscillation tests |
| Observation timeout | `<fill in>` | s | Measured sensor and inference latency |
| LiDAR timeout | `<fill in>` | s | Measured point-cloud rate and jitter |
| LiDAR protective distance | `<fill in>` | m | Robot footprint and measured braking margin |
| Search timeout | `<fill in>` | s | Test-area and rotation limits |

## 10. Evaluate and save evidence

Use repeated trials with the same documented setup when comparing the baseline and an improved version. Record the number of trials and include failures.

For each trial:

1. Mark the robot and target starting poses.
2. Start logging and record the configuration and code revision.
3. Begin the task without manually correcting the robot.
4. Stop the trial on success, timeout, safety intervention, or controller failure.
5. Measure front clearance independently after the robot is stationary.
6. Record the outcome and failure reason before resetting the scene.

| Test | What it checks |
| --- | --- |
| Target initially left, right, and centered | Search and alignment behavior |
| Target initially outside the camera view | Search completion |
| Target absent | Bounded search and timeout behavior |
| Multiple matching objects | Consistent selection and identity handling |
| Narrow or partially occluded target | Target-depth association |
| Target disappears during approach | Loss-of-target stopping |
| Delayed or out-of-order inference result | Observation rejection |
| Depth stream stops or contains invalid values | Data freshness handling |
| Different lighting and distractors | Recognition robustness |
| Obstacle in the route, if supported | Obstacle stopping or avoidance |

For each trial, save:

- Target category, initial pose, scene conditions, and configuration.
- Success or failure with a reason.
- Time from search start to stable stop.
- Independently measured final front clearance.
- Minimum observed clearance, when measurable.
- False detections, target losses, and alignment oscillations.
- Video and relevant ROS logs or bag files.
- Code revision and model/prompt version.

Use a table such as:

| Version | Trials | Successes | Median completion time | Final clearance range | Failure notes |
| --- | --- | --- | --- | --- | --- |
| Baseline | Not measured | — | — | — | — |
| Improved | Not measured | — | — | — | — |

Do not claim an improvement from a single successful video. Compare time together with success rate and stopping accuracy.

## 11. Structure the report

1. **System overview:** perception, depth association, control, and ROS interfaces.
2. **Reference limitations:** which problems you addressed and why.
3. **Implementation:** core code and parameter choices.
4. **Timing and failure handling:** stale observations, missing depth, and target loss.
5. **Evaluation:** repeated trials, quantitative results, and failure analysis.
6. **Optional extensions:** what was implemented and its measured benefit.
7. **Remaining limitations:** unsupported scenes, sensor limitations, and future work.

Distinguish implemented behavior from proposed work. Save the final code, configuration, and robot execution recordings before ending the session. After saving results and stopping the robot, shut down the Orin using the lab's instructed shutdown procedure.

### Final submission checklist

- [ ] The report uses the official lab title and required format.
- [ ] All four prerequisite-lab Checkpoints were verified by the TA and their Questions were answered.
- [ ] The implemented behavior is clearly separated from proposed future work.
- [ ] Topic names, frames, QoS settings, parameters, and model version are documented.
- [ ] Results include the number of trials, failures, and independently measured clearances.
- [ ] Figures and videos identify the tested code revision.
- [ ] Safety interventions and unsuccessful trials are reported.
- [ ] Code, configuration, logs, and recordings are backed up.

## Scope and attribution

This guide is based on the supplied DASE7143 Bunker Mini prerequisite lab, target-search exercise, and `UGVTrashCenterNode` Python scaffold. It assumes the prerequisite LiDAR, RealSense D555, chassis, and `simple_vlm` Checkpoints have been completed. It is an improvement proposal, not an official replacement for the course handout or grading requirements.

Repository: [JINGERGER/DASE7143-LAB3-REFERENCE](https://github.com/JINGERGER/DASE7143-LAB3-REFERENCE)

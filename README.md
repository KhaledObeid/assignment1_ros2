This repository contains three packages that implement bumpgo assignment. All are present in src (the codes for the assignment are t1, t2a, t2b not the others)

    t1: Basic reactive control

    t2a: Closed-loop turning based on free-space detection

    t2b: Enhanced behavior with stuck detection and recovery

t1 – Basic Reactive Control

    Function:
    Implements a simple state machine with FORWARD, BACK, TURN, and STOP states.

    How It Works:

        Subscribes to laser scans and commands forward speed.

        Checks the center laser reading for obstacles.

        Switches states to turn or stop if an obstacle is detected.

    Usage:

        Launch the node and verify that the robot reacts (stops, turns, or backs up) based on obstacles.

t2a – Closed-Loop Reactive Turning

    Function:
    Improves t1 by continuously computing the free space from lateral sectors.

    How It Works:

        Scans the left (+5° to +90°) and right (–90° to –5°) sectors.

        Determines the angle with the maximum free space.

        Uses a proportional controller while moving forward to steer toward that free space.

    Usage:

        Run the node so that the robot dynamically adjusts its heading based on the current sensor feedback.

t2b – Stuck Detection and Recovery

    Function:
    Extends t2a by adding stuck-detection and recovery behaviors.

    How It Works:

        Monitors if the free-space values (range and angle) remain nearly constant over time.

        If detected stuck for a preset period, the node switches to a STUCK state.

        In the STUCK state, the robot backs up for a set time before reattempting turning.

    Usage:

        Use this package for environments where the robot may get trapped in local minima. The node will automatically initiate recovery if stuck.

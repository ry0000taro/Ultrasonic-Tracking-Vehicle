# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

This repository is currently **empty** (git initialized, no commits, no source files yet). This CLAUDE.md is a skeleton based on the project's concept and specification documents (`2-5班_コンセプト_移動台車.pdf` and `2-5班_仕様書_移動台車.xlsx`). It should be regenerated/expanded once source code exists, since there are no build/lint/test commands or real architecture to document yet.

## Project overview

An ultrasonic-sensor-based tracking mobile cart (移動台車) built by team 2-5. Purpose: follow a person to assist with carrying luggage.

Core behavior required by spec:
- Detect distance to a target using ultrasonic sensors and follow it, maintaining a software-configurable follow distance.
- Stop if the target comes within a configurable minimum distance, or if the target is not detected for a configurable timeout (seconds).
- Show current state on an LED panel: green "追従中" (following), yellow "停止中" (stopped), red "エラー" (error).
- Sound a buzzer when the target is lost or an obstacle is detected / on error.
- Electrical/software switch to start and stop the cart.

Key hardware/performance constraints from the spec sheet:
- 3 wheels (2 front, 1 rear).
- Footprint ≤ 400×400 mm (W×D), weight ≤ 3.00 kg.
- Max payload 3 kg; must reach max speed while loaded.
- Battery powered; uses the course-supplied DC motor + encoder (additional actuators/sensors may be added).
- Max speed: 0.5 m/s; speed accuracy: ±10%; stop-position accuracy: ±50 mm.
- Exposed edges/corners must be chamfered or rounded (safety).
- Operates on flat indoor floor (研究実験棟2) and on a work desk surface.

## Commands

Not yet defined — no build system, package manifest, or test framework exists in this repository yet. Add this section once source code (e.g., Arduino/ESP32/microcontroller firmware, or a host-side control app) is added.

## Architecture

Not yet defined — no source files exist. Once implementation begins, this section should describe the actual module breakdown (e.g., sensor reading/filtering, follow/stop control loop, LED state display, buzzer alerts, motor driver) based on the real code rather than the spec alone.

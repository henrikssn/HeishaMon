# Meisha Heat Pump Control

Autonomous local heating, domestic hot water (DHW), cooling, and evaporator-protection control for a single-zone Panasonic Aquarea M-series heat pump.

## Language

### Temperatures & Targets

**Room Target**:
The desired indoor room temperature (°C) set by the user, which also serves as the origin of the heating curve and the baseline for calculating degree-hours.
_Avoid_: Target temperature, indoor setpoint, desired temp

**Curve Target**:
The ideal continuous outlet water temperature (°C) computed from the 24-hour outdoor ambient average, the heating curve parameters (start and angle), and the daily room temperature offset.
_Avoid_: Heating curve setpoint, flow target, compensation target

**Power Target**:
The integer water temperature (°C) obtained by trimming the Curve Target with the slow integral controller (PID) output; represents the steady-state operating point that Bonusgrad and Malusgrad step toward.
_Avoid_: PID target, adjusted curve target

**Heat Pump Setpoint**:
The integer Zone 1 water temperature (°C) commanded to the heat pump by the controller.
_Avoid_: Flow setpoint, Z1 request temp, climate temperature

**Heat Pump Target**:
The effective target water temperature (°C) reported by the heat pump itself that the compressor is actively chasing.
_Avoid_: Main target temp, internal target, actual target

### Heating Budget & Modulation

**Heating Day**:
The 24-hour control cycle running from 09:00 to 09:00 the following morning over which the daily heat budget is planned and consumed.
_Avoid_: Calendar day, daily cycle

**Degree-Hour Budget**:
The total heat energy (in Kelvin-hours, Kh) required over a Heating Day if the heat pump ran continuously for 24 hours at the Curve Target above the Room Target.
_Avoid_: Degree-hours target, daily heat target, Kh target

**Degree-Hours Day**:
The cumulative Kelvin-hours (Kh) of heat delivered since the start of the current Heating Day, integrated from the excess of outlet water temperature over the Room Target while water is flowing to the room loop.
_Avoid_: Daily integral, accumulated degree-hours

**Bonusgrad**:
A +1 K upward step of the Heat Pump Setpoint applied either to ramp toward the Power Target once the outlet water catches up, or to prevent the compressor from short-cycling when water temperature overshoots the Heat Pump Setpoint.
_Avoid_: Boost degree, step up

**Malusgrad**:
A −1 K downward step of the Heat Pump Setpoint applied when the Heat Pump Setpoint is above the Power Target and outlet water temperature permits stepping down without stopping the compressor.
_Avoid_: Step down, setback degree

**Change Lock**:
The cooldown period after a Bonusgrad or Malusgrad adjustment during which further automatic ramp steps are held off to let the water loop stabilize.
_Avoid_: Cooldown timer, lock timestamp

**Night Mode**:
The idle operating state entered once Degree-Hours Day exceeds the Degree-Hour Budget, where the Heat Pump Setpoint is dropped to 25 °C and the heat pump is switched off after 4 hours.
_Avoid_: Night setback, budget-reached mode

**Shadow Mode**:
An observation-only operating mode where the on-device controller computes all averages, targets, budgets, and proposed heat pump commands and publishes them to MQTT without actuating the heat pump.
_Avoid_: Dry-run mode, test mode, passive mode

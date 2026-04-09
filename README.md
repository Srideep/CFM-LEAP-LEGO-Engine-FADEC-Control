#FADEC Control System

**FADEC is a fault-tolerant, gain-scheduled, constrained feedback control system for a strongly nonlinear plant**, where the plant is the gas turbine engine, the measurements are a limited sensor set, the commands are things like fuel flow and variable geometry, and the controller must keep the engine inside safety limits while still giving the pilot the requested thrust. Your document sketches exactly that architecture: sensors, control laws, actuators, dual lanes, and degraded/failure states.  ([Federal Aviation Administration][1])

## 1. The engine is the **plant**

In control notation, the engine is not “just a machine,” it is the plant:

[
\dot{x}=f(x,u,d,\theta), \qquad y=g(x,u,d,\theta)+v
]

where:

* (x) = internal engine states, such as spool speeds and thermodynamic states
* (u) = actuator commands, especially fuel flow, and in many engines variable stator vanes, bleed valves, or nozzle area
* (d) = disturbances / operating condition, such as altitude, Mach number, ambient temperature
* (\theta) = health/degradation parameters, because the engine changes as it ages
* (y) = measured outputs from sensors
* (v) = measurement noise / uncertainty

NASA describes modern engine control exactly in this spirit: the engine must work across a wide flight envelope, thrust itself is not directly measurable in flight, several limiting quantities are not directly measurable either, and only a modest number of sensors are available in a harsh environment. ([NASA Technical Reports Server][2])

From an engineering perspective, that means FADEC lives in the classic EE world of a **nonlinear, time-varying, partially observed system**. It is much closer to flight controls or power electronics control than to ordinary business software. ([NASA Technical Reports Server][2])

## 2. The pilot does not command fuel directly, the pilot commands a **performance target**

Your document’s “pilot throttle → FADEC → engine” block is the human-readable version of a reference-tracking problem. The pilot moves the power lever angle, and FADEC converts that into a reference (r), usually in terms of a thrust-related variable such as **N1** or **EPR**, because actual thrust is hard to measure directly in flight. NASA notes that thrust is not directly measurable during flight, so the controller regulates a sensed quantity correlated with thrust, commonly fan speed (N1) or engine pressure ratio (EPR). SKYbrary defines EPR as turbine discharge pressure divided by compressor inlet pressure and notes that it is used as a thrust-related measure; N1 is the alternate thrust-setting method. ([NASA Technical Reports Server][3])

So mathematically, the command path looks like this:

[
r = h(\text{throttle},\ \text{altitude},\ \text{Mach},\ \text{temperature}, \ldots)
]

The control problem is then to drive the measured controlled variable (y_c) toward (r). In your PDF, that is the intuition behind the “EPR schedules” and “N schedules.”  ([NASA Technical Reports Server][3])

## 3. The core loop is closed-loop feedback

At the heart of FADEC is a feedback loop:

[
e(t)=r(t)-y_c(t)
]

[
u_{\text{reg}}(t)=K_p(\rho)e(t)+K_i(\rho)\int e(t),dt
]

where (\rho) is the operating condition or scheduling vector, such as altitude, Mach, fan speed, or core speed. NASA’s generic commercial-engine control work explicitly uses PI-style regulators and explains that the gains are **scheduled** across the operating envelope rather than fixed globally. ([NASA Technical Reports Server][3])

Why scheduled gains? Because a turbofan is highly nonlinear. The same fuel increment does not produce the same response at sea level static, climb, cruise, hot day, cold day, or partially degraded health. NASA describes gain scheduling as breaking the nonlinear engine into local linear subsystems, tuning a controller at each breakpoint, then interpolating the gains during operation. ([NASA Technical Reports Server][3])

That is one of the best “mental bridges” from software to controls:

* ordinary software likes one rule set everywhere
* FADEC uses **different local rule sets** depending on where the engine is operating

It is like switching from one universal function to a stitched map of locally valid linear models. ([NASA Technical Reports Server][3])

## 4. FADEC is not only a tracker, it is a **constraint manager**

If FADEC only tracked (r), it could easily destroy the engine during aggressive transients. So the real control problem is:

[
\text{track the commanded thrust proxy}
]
subject to
[
N_1 \le N_{1,\max},\quad N_2 \le N_{2,\max},\quad T \le T_{\max},\quad \text{stall margin} \ge \text{minimum},\quad \text{fuel flow within safe accel/decel bounds}
]

NASA describes this in practical terms: the regulated fuel-flow command is filtered through **protection logic** that enforces maximum speed, combustor pressure, acceleration and deceleration schedules, and minimum ratio-unit limits so the engine stays safe and stable. It also notes that these limits are implemented with “select low” and “select high” style logic, meaning the final fuel command is whichever safe limit is most restrictive at that instant. ([NASA Technical Reports Server][2])

So the true command is closer to:

[
u_{\text{cmd}}=\Pi_{\mathcal{U}*{safe}}(u*{\text{reg}})
]

where (\Pi_{\mathcal{U}_{safe}}) means “project the desired command into the safe allowable region.”

That is the engineering soul of FADEC. It is not merely asking, “How do I get more thrust?” It is asking, “What is the **maximum safe command** that still moves me toward the requested thrust?” ([NASA Technical Reports Server][3])

## 5. Why EPR / N1 schedules matter mathematically

Your PDF mentions EPR mode and N mode. The engineering meaning is that FADEC chooses a **controlled variable** that correlates well with thrust. NASA notes that EPR correlates with airflow through the engine and fan speed (N1) correlates well with thrust in high-bypass turbofans, so either can be used as the primary controlled variable depending on engine architecture and mode. ([NASA Technical Reports Server][3])

So, instead of “throttle → fuel directly,” the logic is more like:

[
\text{throttle} \to r_{EPR}\ \text{or}\ r_{N1}
\to \text{regulator}
\to \text{fuel flow}
\to \text{engine response}
]

That matters because it turns an ugly thermodynamic problem into a more manageable control problem. You pick a measurable proxy that behaves monotonically enough with thrust, then regulate that proxy robustly. ([NASA Technical Reports Server][3])

## 6. The engine is only **partially observed**, so estimation matters

This is where the math gets especially elegant.

NASA points out that typical engines may have only about 6 to 8 useful sensors available for estimating around 10 health parameters, which makes the problem **underdetermined**. In advanced model-based engine control, this is handled with linear state-space models, reduced-order tuning parameters, and Kalman filtering / EKF-based observers. ([NASA Technical Reports Server][4])

NASA gives the linearized model in the form:

[
x_{k+1}=Ax_k+Bu_k+Lh_k+w_k
]

[
y_k=Cx_k+Du_k+Mh_k+v_k
]

[
z_k=Fx_k+Gu_k+Nh_k
]

where (h_k) represents engine-health effects, (y_k) are measured outputs, and (z_k) are auxiliary unmeasured outputs of interest. ([NASA Technical Reports Server][4])

That is a lovely systems-engineering idea: FADEC is not just reacting to raw sensors, it can run a compact **onboard model** of the engine and compare model-predicted behavior to measured behavior.

Then the observer step is conceptually:

[
\hat{x}*{k}^{+}=\hat{x}*{k}^{-}+K_k(y_k-\hat{y}_{k}^{-})
]

That residual term, (y_k-\hat{y}_{k}^{-}), is the little whisper between reality and prediction. If it stays small, life is calm. If it grows, something is drifting, degrading, or failing. NASA uses this same residual-based idea for fault detection and health estimation. ([NASA Technical Reports Server][5])

## 7. Health monitoring is basically **diagnostics + prognostics through residuals**

Your PDF lists diagnostics, prognostics, and adaptive behavior. In control/estimation language, this usually means comparing measured outputs to model-expected outputs and tracking the mismatch over time. NASA describes fault detection using residuals between sensed outputs and baseline-model estimates, with a weighted sum of squared residuals used for detection and weighted comparisons used for fault classification. ([NASA Technical Reports Server][5])

Mathematically, that looks like:

[
r_k = y_k - \hat{y}_k
]

[
J_k = r_k^\top R^{-1} r_k
]

If (J_k) crosses a threshold, the system suspects a fault or abnormal degradation. That is the same structural idea you would see in signal processing, radar tracking, or model-based fault detection. ([NASA Technical Reports Server][5])

So in your “mental model” mapping:

* **sensors** become the measurement vector (y)
* **monitors** become residual generators / fault detectors
* **prognostics** become slowly varying parameter estimation on (\theta) or (h)
* **adaptive logic** becomes retuning control or limits based on estimated engine condition ([NASA Technical Reports Server][5])

## 8. Dual lanes mean this is also a **reliability and fault-tolerance problem**

Your PDF’s Lane A / Lane B and Markov-state slides are the dependability side of the same machine. FAA guidance defines FADEC as a control system with full-range electronic authority over thrust/power, and notes that certified systems may use two identical channels for full operational capability after one channel failure, or alternate backup architectures. FAA also requires safety assessment and explicit demonstration of fault accommodation logic, including handling of single and dual failures of control-system signals.  ([Federal Aviation Administration][1])

Mathematically, once you have redundant channels, you stop thinking only in transfer functions and start thinking in **state-transition reliability models**. NASA’s Markov-model tutorials describe Markov models as useful for fault-tolerant systems because they represent system behavior as states and transitions among them. 

So your PDF’s healthy / degraded / shutdown boxes can be viewed as a Markov chain:

[
\dot{p}(t)=p(t)Q
]

or in discrete time,

[
p_{k+1}=p_k P
]

where (p) is the probability distribution over states like:

* fully healthy
* one monitor failed
* one command lane failed
* cross-monitor failure
* uncontrollable / shutdown state  

Then quantities like:

* **dispatchable probability**
* **controllable probability**
* **shutdown probability**
* **mean time to failure**

drop out of that model. In other words, the Markov diagram is the reliability engineer’s equivalent of a state machine. It answers, “If I lose this monitor now, what is the probability that one more fault pushes me into a non-dispatchable or shutdown condition?”  

## 9. A very compact engineering abstraction

If I compress the whole thing into one line of control theory, FADEC is approximating this problem:

[
\min_{u(t)} \int_0^T \Big[w_1(r-y_c)^2 + w_2(\Delta u)^2 + w_3(\text{life usage})\Big]dt
]

subject to

[
\dot{x}=f(x,u,d,\theta), \qquad g(x,u,d,\theta)\le 0
]

where the inequality constraints represent overspeed, overtemperature, surge margin, flameout margin, pressure limits, and other certification-critical boundaries. That equation is an **engineering interpretation**, not a claim that every production FADEC literally solves that exact optimal-control problem online, but it captures the design intent very well. The NASA control papers support the pieces of that view: nonlinear plant, scheduled regulators, limit protection, health estimation, and life-impact awareness during transients. ([NASA Technical Reports Server][6])

## 10. Best mental model, now in EE language

So the mapping I gave earlier becomes:

* **Sensors** = measurement vector (y)
* **Actuators** = control vector (u)
* **Control laws** = scheduled regulator (K(\rho))
* **Mode switching** = supervisory hybrid logic
* **Monitors** = residual-based fault detection
* **Dual lanes** = redundant fault-tolerant architecture
* **Markov model** = probabilistic state machine for dispatchability and shutdown risk
* **Health monitoring** = online parameter/state estimation under sparse sensing  ([Federal Aviation Administration][1])

So from an engineering point of view, FADEC is not “engine software” in the casual sense.

It is a **real-time embedded control system sitting at the intersection of nonlinear dynamics, estimation theory, fault tolerance, reliability engineering, and certification-driven safety logic**. That is why the slides feel like a blend of control systems, avionics, and systems safety, because that is exactly what FADEC is. ([Federal Aviation Administration][1])

I can also do a next-pass version focused specifically on **the equations behind EPR/N1 control and limiter selection**, or on **the Markov reliability math in pages 13 to 16**.

[1]: https://www.faa.gov/documentLibrary/media/Advisory_Circular/33.28-1.pdf "Advisory Circle"
[2]: https://ntrs.nasa.gov/api/citations/20130013439/downloads/20130013439.pdf "Microsoft Word - E-18277-1_TM"
[3]: https://ntrs.nasa.gov/api/citations/20100037768/downloads/20100037768.pdf "untitled"
[4]: https://ntrs.nasa.gov/api/citations/20160002248/downloads/20160002248.pdf "Microsoft Word - E-19199TM"
[5]: https://ntrs.nasa.gov/api/citations/20140016832/downloads/20140016832.pdf "Slide 1"
[6]: https://ntrs.nasa.gov/api/citations/20150000702/downloads/20150000702.pdf "Propulsion Controls at  NASA Lewis"

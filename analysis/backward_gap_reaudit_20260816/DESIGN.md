# Design brief

## Purpose

Produce a technical correction and re-audit for a robotics researcher. The
report must separate observations, inferences, and untestable hypotheses. It
must explicitly retract conclusions contradicted by the two motor logs and
must not treat the repository's current policy checkpoint as the deployed
checkpoint.

## Audience and tone

The audience is a robotics RL practitioner. Use compact engineering language,
SI units, explicit sample counts, and causal caveats. Avoid decorative prose.

## Visual system

- White background with restrained blue/teal accents.
- One grouped bar chart compares the six near-static HFE residuals in real
  hardware and in the current MuJoCo model.
- Tables carry exact configuration comparisons, corrected claims, and the
  prioritized experiment matrix.
- Use red only for a verified contract mismatch, amber for an unverified but
  high-value hypothesis, and green for a hypothesis contradicted by evidence.

## Reading order

1. Corrected conclusion and four headline facts.
2. What the two logs do and do not prove.
3. Static logged-target replay.
4. MuJoCo versus IsaacGym non-reward configuration comparison.
5. Root-cause ranking and minimum safe experiments.
6. Limitations and provenance.

## Accessibility

Do not encode status by color alone. Every status appears as text. Chart labels
include joint names and radian units.

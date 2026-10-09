# Task Intent: New Sensor Migration

> Superseded historical intent. The approved follow-up expanded the original raw-acquisition scope into a complete ADC detection migration and retirement of the legacy sensor path.

- Final requested outcome: implement DAC + ADC DMA detection in `SHOOT_cmzy_REV081`, integrate shot events with thermal/LED/CAN, and retire VCNL4040/`shoot_detect` runtime and source references.
- Final scope: PA4/PA5 DAC, PA1/PA3 ADC DMA, physical front/rear naming, calibrated baseline/threshold detection, peak-time optional rear matching, unconditional front events, CAN retry, thermal/LED behavior, documentation, and verification.
- Final non-goals: hardware flashing and physical board validation.
- Final owner boundary: `ir_acquisition` owns physical mapping and sampling; `ir_detection` owns algorithm/diagnostics; `main` owns application effects and retry scheduling; `can_protocol` owns wire encoding; legacy VCNL4040/`shoot_detect` owners are deleted.
- Historical baseline and raw-only decisions remain preserved above for traceability only.
- Stop condition: build, replay, static retirement checks, and diff checks pass; hardware validation is explicitly reported pending.

//! Per-phase timings of a physics step, published as Godot custom performance monitors
//! under `rapier/*` so they graph next to `TIME_PHYSICS_PROCESS`.
//!
//! Enabled by the `profiling` feature; without it every function here is an inlined no-op,
//! so instrumented call sites need no `cfg` of their own.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Span {
    Step,
    Solver,
    ActiveSync,
    Events,
    EventsCollision,
    EventsContactForce,
    Fluids,
    FlushQueries,
    FlushCollect,
    FlushDispatch,
    /// Only the callable invocation inside a dispatched body-state sync, so the cost of
    /// crossing into Godot can be told apart from the bookkeeping around it. Dispatch minus
    /// this is what optimising our own code could ever recover.
    FlushCall,

    // Rapier's own per-stage timers, drained out of `PhysicsPipeline::counters` after each
    // step. Split by whether the stage has a parallel implementation, because their ratio is
    // what caps the speedup any thread count can reach (see `report_if_due`).
    /// Serial: delayed wake-ups, user-change propagation, island joins, forward kinematics.
    StepUserChanges,
    /// Serial: island construction and body activation/deactivation.
    StepIslands,
    /// Serial: solver-facing structure maintenance (contact graph reconciliation).
    StepConstraints,
    /// Parallel: `par_chunks` over active bodies (energy, effective forces).
    StepUpdate,
    /// Parallel: BVH refit and pair traversal.
    StepBroadPhase,
    /// Partly parallel: contact computation and intersection updates.
    StepNarrowPhase,
    /// Parallel: the staged island solver's worker fan-out.
    StepSolver,
    /// Partly parallel: CCD sweeps (fans out only above a body-count threshold).
    StepCcd,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Gauge {
    ActiveBodies,
    TotalBodies,
    ContactForceEvents,
    ContactForceColliders,
    /// Callables invoked during flush dispatch — the per-frame count of crossings into Godot.
    FlushCallbacks,
}

#[cfg(feature = "profiling")]
mod enabled {
    use std::sync::atomic::AtomicU64;
    use std::sync::atomic::Ordering;
    use std::time::Instant;

    use godot::classes::Performance;
    use godot::prelude::*;

    use super::*;

    const SPANS: [(Span, &str); 19] = [
        (Span::Step, "rapier/step_ms"),
        (Span::Solver, "rapier/solver_ms"),
        (Span::ActiveSync, "rapier/active_sync_ms"),
        (Span::Events, "rapier/events_ms"),
        (Span::EventsCollision, "rapier/events_collision_ms"),
        (Span::EventsContactForce, "rapier/events_contact_force_ms"),
        (Span::Fluids, "rapier/fluids_ms"),
        (Span::FlushQueries, "rapier/flush_queries_ms"),
        (Span::FlushCollect, "rapier/flush_collect_ms"),
        (Span::FlushDispatch, "rapier/flush_dispatch_ms"),
        (Span::FlushCall, "rapier/flush_call_ms"),
        (Span::StepUserChanges, "rapier/stage_user_changes_ms"),
        (Span::StepIslands, "rapier/stage_islands_ms"),
        (Span::StepConstraints, "rapier/stage_constraints_ms"),
        (Span::StepUpdate, "rapier/stage_update_ms"),
        (Span::StepBroadPhase, "rapier/stage_broad_phase_ms"),
        (Span::StepNarrowPhase, "rapier/stage_narrow_phase_ms"),
        (Span::StepSolver, "rapier/stage_solver_ms"),
        (Span::StepCcd, "rapier/stage_ccd_ms"),
    ];
    /// Rapier stages with no parallel implementation. Their share of the frame — together with
    /// everything outside the step (see `report_if_due`) — is the serial fraction.
    const SERIAL_STAGES: [Span; 3] = [
        Span::StepUserChanges,
        Span::StepIslands,
        Span::StepConstraints,
    ];
    /// Rapier stages that fan out across the pool.
    const PARALLEL_STAGES: [Span; 5] = [
        Span::StepUpdate,
        Span::StepBroadPhase,
        Span::StepNarrowPhase,
        Span::StepSolver,
        Span::StepCcd,
    ];
    const GAUGES: [(Gauge, &str); 5] = [
        (Gauge::ActiveBodies, "rapier/active_bodies"),
        (Gauge::TotalBodies, "rapier/total_bodies"),
        (Gauge::ContactForceEvents, "rapier/contact_force_events"),
        (
            Gauge::ContactForceColliders,
            "rapier/contact_force_colliders",
        ),
        (Gauge::FlushCallbacks, "rapier/flush_callbacks"),
    ];

    struct Accum {
        nanos: AtomicU64,
        count: AtomicU64,
    }
    impl Accum {
        const fn new() -> Self {
            Self {
                nanos: AtomicU64::new(0),
                count: AtomicU64::new(0),
            }
        }
    }

    static SPAN_ACCUM: [Accum; SPANS.len()] = [const { Accum::new() }; SPANS.len()];
    static GAUGE_VALUE: [AtomicU64; GAUGES.len()] = [const { AtomicU64::new(0) }; GAUGES.len()];

    /// Drains a span, returning its accumulated milliseconds and how many times it was entered.
    /// Reading is what defines the averaging window, so an unread span just accumulates over more
    /// steps rather than growing without bound.
    fn take_totals(span: Span) -> (f64, u64) {
        let accum = &SPAN_ACCUM[span as usize];
        let nanos = accum.nanos.swap(0, Ordering::Relaxed);
        let count = accum.count.swap(0, Ordering::Relaxed);
        (nanos as f64 / 1.0e6, count)
    }

    /// Mean cost of one entry into the span. Note this is *per entry*, not per step: spans
    /// entered many times per step (`FlushCall`, once per dispatched callable) read far smaller
    /// than their per-step cost. The console report uses per-step totals instead.
    fn take_mean_ms(span: Span) -> f64 {
        let (ms, count) = take_totals(span);
        if count == 0 { 0.0 } else { ms / count as f64 }
    }

    pub fn record(span: Span, nanos: u64) {
        let accum = &SPAN_ACCUM[span as usize];
        accum.nanos.fetch_add(nanos, Ordering::Relaxed);
        accum.count.fetch_add(1, Ordering::Relaxed);
    }

    pub fn set_gauge(gauge: Gauge, value: u64) {
        GAUGE_VALUE[gauge as usize].store(value, Ordering::Relaxed);
    }

    #[must_use = "the span is measured until this guard drops"]
    pub struct Timer {
        span: Span,
        start: Instant,
    }
    impl Drop for Timer {
        fn drop(&mut self) {
            record(self.span, self.start.elapsed().as_nanos() as u64);
        }
    }

    pub fn scope(span: Span) -> Timer {
        Timer {
            span,
            start: Instant::now(),
        }
    }

    pub fn register_monitors() {
        let mut performance = Performance::singleton();
        for (span, id) in SPANS {
            let name = StringName::from(id);
            if performance.has_custom_monitor(&name) {
                continue;
            }
            let callable = Callable::from_fn(id, move |_args: &[&Variant]| take_mean_ms(span));
            performance.add_custom_monitor(&name, &callable);
        }
        for (gauge, id) in GAUGES {
            let name = StringName::from(id);
            if performance.has_custom_monitor(&name) {
                continue;
            }
            let callable = Callable::from_fn(id, move |_args: &[&Variant]| {
                GAUGE_VALUE[gauge as usize].load(Ordering::Relaxed) as f64
            });
            performance.add_custom_monitor(&name, &callable);
        }
        godot_print!("Rapier profiling enabled: see Debugger -> Monitors -> rapier/*");
    }

    /// Headless runs have no debugger polling the monitors, so summarise to the console too.
    pub fn report_if_due() {
        static STEPS: AtomicU64 = AtomicU64::new(0);
        const EVERY: u64 = 120;
        if STEPS.fetch_add(1, Ordering::Relaxed) % EVERY != EVERY - 1 {
            return;
        }
        let physics_frame_ms = Performance::singleton()
            .get_monitor(godot::classes::performance::Monitor::TIME_PHYSICS_PROCESS)
            * 1000.0;

        // Drain every span once up front (draining resets the accumulator, so the summary below
        // must reuse these values) and normalise to *per step*, not per entry: a span entered
        // once per dispatched callable has to be comparable with one entered once per step
        // before any of it can be added up.
        let mut per_step = [0.0f64; SPANS.len()];
        let mut entries = [0.0f64; SPANS.len()];
        for (span, _) in SPANS {
            let (ms, count) = take_totals(span);
            per_step[span as usize] = ms / EVERY as f64;
            entries[span as usize] = count as f64 / EVERY as f64;
        }
        let mean = |span: Span| per_step[span as usize];

        let mut line = format!("[rapier] physics_frame={physics_frame_ms:.3}ms");
        for (span, id) in SPANS {
            let label = id.trim_start_matches("rapier/").trim_end_matches("_ms");
            line.push_str(&format!(" {label}={:.3}", mean(span)));
            // Spans entered more than once per step: show the rate, since the per-step total
            // alone hides whether it is one expensive call or thousands of cheap ones.
            if entries[span as usize] > 1.5 {
                line.push_str(&format!("(x{:.0})", entries[span as usize]));
            }
        }
        for (gauge, id) in GAUGES {
            let label = id.trim_start_matches("rapier/");
            line.push_str(&format!(
                " {label}={}",
                GAUGE_VALUE[gauge as usize].load(Ordering::Relaxed)
            ));
        }
        godot_print!("{line}");
        report_scaling(&per_step);
    }

    /// The measured Amdahl ceiling: how much of the physics work can fan out at all, and the
    /// best speedup any thread count could therefore reach. `Step` covers everything from the
    /// pipeline step through event dispatch, and `FlushQueries` the callback crossing after it;
    /// both are wall-clock, so subtracting the parallel stages leaves the true serial residue.
    ///
    /// Rapier's stage timers are disjoint except that broad- and narrow-phase nest inside
    /// collision detection, so the glue between them lands in the residue: this reads as a
    /// slightly *pessimistic* ceiling, never an optimistic one.
    fn report_scaling(per_step: &[f64; SPANS.len()]) {
        let mean = |span: Span| per_step[span as usize];
        let total = mean(Span::Step) + mean(Span::FlushQueries);
        let parallel: f64 = PARALLEL_STAGES.iter().copied().map(mean).sum();
        if total <= 0.0 {
            godot_print!(
                "[rapier] scaling: no timings -- build with the `profiling` feature so rapier's \
                 stage counters are compiled in"
            );
            return;
        }
        // Clamped because the two come from different clocks: the stage timers are summed
        // per-step while `Step` is one wall-clock span, so rounding can push the ratio past 1.
        let serial_frac = (1.0 - parallel / total).clamp(0.0, 1.0);
        let speedup = |n: f64| 1.0 / (serial_frac + (1.0 - serial_frac) / n);

        #[cfg(feature = "parallel")]
        let threads =
            crate::servers::rapier_project_settings::RapierProjectSettings::get_num_threads();
        #[cfg(not(feature = "parallel"))]
        let threads = 1;

        godot_print!(
            "[rapier] scaling: total={total:.3}ms parallel={parallel:.3}ms serial={:.3}ms \
             serial_frac={:.1}% | threads={threads} ceiling={:.2}x (4t={:.2}x 8t={:.2}x inf={:.2}x)",
            total - parallel,
            serial_frac * 100.0,
            speedup(threads as f64),
            speedup(4.0),
            speedup(8.0),
            speedup(f64::INFINITY),
        );
        // `Solver` is the wall-clock span around `PhysicsPipeline::step`; the stage timers sum
        // to the work it accounts for. The remainder is pipeline glue plus whatever the pool
        // costs to hand work to -- the number to watch if worker wake-up latency is suspected.
        let accounted: f64 = SERIAL_STAGES
            .iter()
            .chain(PARALLEL_STAGES.iter())
            .copied()
            .map(mean)
            .sum();
        godot_print!(
            "[rapier] pipeline overhead (Solver span minus stage timers): {:.3}ms",
            (mean(Span::Solver) - accounted).max(0.0)
        );
        // The biggest serial stages, so the summary points at what to attack rather than
        // just how bad it is. Rapier's own serial stages are reported next to ours.
        let mut serial: Vec<(&str, f64)> = SERIAL_STAGES
            .iter()
            .map(|s| (label_of(*s), mean(*s)))
            .chain([
                ("outside_step(flush)", mean(Span::FlushQueries)),
                ("godot_crossing", mean(Span::FlushCall)),
                ("active_sync", mean(Span::ActiveSync)),
                ("events", mean(Span::Events)),
            ])
            .collect();
        serial.sort_by(|a, b| b.1.total_cmp(&a.1));
        let top = serial
            .iter()
            .take(4)
            .map(|(l, ms)| format!("{l}={ms:.3}ms({:.0}%)", ms / total * 100.0))
            .collect::<Vec<_>>()
            .join(" ");
        godot_print!("[rapier] serial hotspots: {top}");
    }

    fn label_of(span: Span) -> &'static str {
        SPANS
            .iter()
            .find(|(s, _)| *s == span)
            .map(|(_, id)| id.trim_start_matches("rapier/").trim_end_matches("_ms"))
            .unwrap_or("?")
    }

    /// Drains one step's worth of rapier stage timers into the spans. Called after every
    /// `PhysicsPipeline::step`, whose counters are reset at the start of the next one.
    pub fn record_step_counters(counters: &rapier::counters::Counters) {
        let nanos = |t: &rapier::counters::Timer| t.time().as_nanos() as u64;
        record(Span::StepUserChanges, nanos(&counters.stages.user_changes));
        record(
            Span::StepIslands,
            nanos(&counters.stages.island_construction_time),
        );
        record(
            Span::StepConstraints,
            nanos(&counters.stages.island_constraints_collection_time),
        );
        record(Span::StepUpdate, nanos(&counters.stages.update_time));
        record(Span::StepBroadPhase, nanos(&counters.cd.broad_phase_time));
        record(Span::StepNarrowPhase, nanos(&counters.cd.narrow_phase_time));
        record(Span::StepSolver, nanos(&counters.stages.solver_time));
        record(Span::StepCcd, nanos(&counters.stages.ccd_time));
        // `counters.cd.ncontact_pairs` and `counters.solver.nconstraints` are never written by
        // rapier 0.35, so they are deliberately not surfaced -- they would read a constant zero.
    }
}

#[cfg(not(feature = "profiling"))]
mod disabled {
    use super::*;

    pub struct Timer;

    #[inline(always)]
    pub fn scope(_span: Span) -> Timer {
        Timer
    }

    #[inline(always)]
    pub fn record(_span: Span, _nanos: u64) {}

    #[inline(always)]
    pub fn set_gauge(_gauge: Gauge, _value: u64) {}

    #[inline(always)]
    pub fn register_monitors() {}

    #[inline(always)]
    pub fn report_if_due() {}

    #[inline(always)]
    pub fn record_step_counters(_counters: &rapier::counters::Counters) {}
}

#[cfg(not(feature = "profiling"))]
pub use disabled::*;
#[cfg(feature = "profiling")]
pub use enabled::*;

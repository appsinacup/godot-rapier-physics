//! Per-phase timings of a physics step, published as Godot custom performance monitors
//! under `rapier/*` so they graph next to `TIME_PHYSICS_PROCESS`.
//!
//! Enabled by the `profiling` feature; without it every function here is an inlined no-op,
//! so instrumented call sites need no `cfg` of their own.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Span {
    Step,
    Solver,
    BeforeActive,
    ActiveSync,
    Events,
    EventsCollision,
    EventsContactForce,
    Fluids,
    FlushQueries,
    FlushCollect,
    FlushDispatch,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Gauge {
    ActiveBodies,
    TotalBodies,
    ContactForceEvents,
    ContactForceColliders,
}

#[cfg(feature = "profiling")]
mod enabled {
    use std::sync::atomic::AtomicU64;
    use std::sync::atomic::Ordering;
    use std::time::Instant;

    use godot::classes::Performance;
    use godot::prelude::*;

    use super::*;

    const SPANS: [(Span, &str); 11] = [
        (Span::Step, "rapier/step_ms"),
        (Span::Solver, "rapier/solver_ms"),
        (Span::BeforeActive, "rapier/before_active_ms"),
        (Span::ActiveSync, "rapier/active_sync_ms"),
        (Span::Events, "rapier/events_ms"),
        (Span::EventsCollision, "rapier/events_collision_ms"),
        (Span::EventsContactForce, "rapier/events_contact_force_ms"),
        (Span::Fluids, "rapier/fluids_ms"),
        (Span::FlushQueries, "rapier/flush_queries_ms"),
        (Span::FlushCollect, "rapier/flush_collect_ms"),
        (Span::FlushDispatch, "rapier/flush_dispatch_ms"),
    ];
    const GAUGES: [(Gauge, &str); 4] = [
        (Gauge::ActiveBodies, "rapier/active_bodies"),
        (Gauge::TotalBodies, "rapier/total_bodies"),
        (Gauge::ContactForceEvents, "rapier/contact_force_events"),
        (
            Gauge::ContactForceColliders,
            "rapier/contact_force_colliders",
        ),
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

    /// Reading is what defines the averaging window, so an unread span just averages over
    /// more steps rather than growing without bound.
    fn take_mean_ms(span: Span) -> f64 {
        let accum = &SPAN_ACCUM[span as usize];
        let nanos = accum.nanos.swap(0, Ordering::Relaxed);
        let count = accum.count.swap(0, Ordering::Relaxed);
        if count == 0 {
            0.0
        } else {
            nanos as f64 / count as f64 / 1.0e6
        }
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
        let mut line = format!("[rapier] physics_frame={physics_frame_ms:.3}ms");
        for (span, id) in SPANS {
            let label = id.trim_start_matches("rapier/").trim_end_matches("_ms");
            line.push_str(&format!(" {label}={:.3}", take_mean_ms(span)));
        }
        for (gauge, id) in GAUGES {
            let label = id.trim_start_matches("rapier/");
            line.push_str(&format!(
                " {label}={}",
                GAUGE_VALUE[gauge as usize].load(Ordering::Relaxed)
            ));
        }
        godot_print!("{line}");
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
}

#[cfg(not(feature = "profiling"))]
pub use disabled::*;
#[cfg(feature = "profiling")]
pub use enabled::*;

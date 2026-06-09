# ADR 0008: In-house indexed ROS1 bag reader (replacing the `rosbag` crate)

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

OpenLORIS bags are bzip2-compressed inside: cafe1-1 is 2.4 GB over ~57 s of recording
(~42 MB/s compressed), and single-core bzip2 decompresses at ~15 MB/s. Reading the bag is
therefore **CPU-bound on decompression**, ~3 minutes per full pass — and the upcoming
RGB-D front-end must eventually consume images *at runtime*, which at these rates needs
~3 cores of decompression running ahead of the consumer.

The bag format itself offers the solution: the index section (`ChunkInfo` records) says
exactly which chunks contain which connections. On cafe1-1, `/scan` messages live in only
2,206 of 6,835 chunks (0.78 of 2.55 GB). But the `rosbag` crate (a) always decompresses
every chunk, (b) exposes no raw compressed chunk payloads, so decompression cannot be
parallelised from outside, and (c) hides the index-to-chunk mapping behind its iterator.

## Decision

`slam-datasets` gets its **own minimal ROS1 v2.0 reader** (`bag.rs`, ~350 lines, no
unsafe), and the `rosbag` dependency is dropped:

1. **Index-first**: parse connections + chunk directory from `index_pos`; never touch a
   chunk that doesn't carry a requested connection.
2. **Raw chunk access**: compressed payloads are read as plain byte ranges, making
   per-chunk **parallel decompression** (rayon) possible — chunks are independent.
3. Decompression via the same `bzip2`/`lz4` crates the `rosbag` crate already pulled in
   (no new transitive dependencies).
4. Unindexed (unfinished) bags are rejected with a pointer to `rosbag reindex` rather
   than supported — every dataset bag in the wild is indexed.

## Consequences

- **Easier:** extraction cost becomes proportional to the *requested* data; runtime
  streaming of RGB-D frames from the bag becomes feasible (parallel decode ahead of the
  consumer); one fewer external dependency.
- **Harder:** we own ~350 lines of format parsing. Mitigation: the format is stable
  (frozen since 2010), the committed `mini.bag` fixture exercises index parsing and
  record walking, and real-bag outputs were verified byte-identical against the previous
  reader's extraction.
- **Risk accepted:** exotic bags (no index, format v1.2) are not supported — they don't
  occur in the datasets we target, and the error message says what to do.

## Alternatives considered

- **Keep `rosbag`, skip chunks via `seek()`:** possible for skipping, but raw payloads
  stay hidden, so parallel decompression — the dominant win and a hard requirement for
  runtime RGB-D streaming — remains impossible. Rejected.
- **Patch the crate upstream:** worthwhile eventually, but the surface we need (index +
  raw chunks + parallel decode) is a different API shape, not a patch.
- **Recompress bags to lz4/uncompressed once at download:** helps locally but doubles
  disk (bags are 2–33 GB) and silently diverges the cached artifact from the published
  dataset. Rejected as the primary mechanism.

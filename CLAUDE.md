# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

libCacheSim is a high-performance cache simulation library written in C/C++. It provides:
- A cache simulator capable of 20M+ requests/sec
- A trace analyzer for cache traces
- A library for building custom cache simulators

## Build Commands

```bash
# Quick install (installs dependencies and builds)
cd scripts && bash install_dependency.sh && bash install_libcachesim.sh

# Manual build
mkdir _build && cd _build
cmake -G Ninja .. && ninja
sudo ninja install

# Debug build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug .. && ninja
```

**Dependencies**: glib, tcmalloc (google-perftools), zstd, cmake, ninja

## Running Tests

```bash
cd _build
ctest                          # Run all tests
ctest -V                       # Verbose output
ctest -R testEvictionAlgo      # Run specific test by name
./bin/testEvictionAlgo         # Run test executable directly
```

Test files are in `/test/` directory.

## Using the CLI Tools

```bash
# Basic cache simulation
./bin/cachesim ../data/trace.vscsi vscsi lru 1gb

# Multiple cache sizes
./bin/cachesim ../data/trace.vscsi vscsi lru 1mb,16mb,256mb,8gb

# CSV trace with format specification
./bin/cachesim ../data/trace.csv csv lru 1gb -t "time-col=2, obj-id-col=5, obj-size-col=4"

# Debug with GDB
./scripts/debug.sh -- data/cloudPhysicsIO.vscsi vscsi lru 100mb
```

## Architecture

### Core Modules

- **`/libCacheSim/cache/eviction/`** - Eviction algorithms (40+): FIFO, LRU, LFU, ARC, S3-FIFO, SIEVE, QDLP, etc.
- **`/libCacheSim/cache/admission/`** - Admission algorithms: adaptsize, bloomfilter, prob, size
- **`/libCacheSim/cache/prefetch/`** - Prefetch algorithms: OBL, Mithril, PG
- **`/libCacheSim/traceReader/`** - Trace file readers (CSV, binary, VSCSI formats)
- **`/libCacheSim/dataStructure/`** - Hash tables, consistent hashing (ketama)
- **`/libCacheSim/profiler/`** - LRU profiler and performance analysis
- **`/libCacheSim/mrcProfiler/`** - Miss ratio curve generation (SHARDS sampling)

### Binary Tools (`/libCacheSim/bin/`)

- **cachesim/** - Main CLI simulator
- **traceAnalyzer/** - Trace analysis tool
- **traceUtils/** - Trace manipulation utilities
- **mrcProfiler/** - MRC profiler

### Public API

Main header: `<libCacheSim.h>`

```c
// Open trace
reader_t *reader = open_trace("trace.vscsi", VSCSI_TRACE, NULL);

// Create cache
common_cache_params_t params = {.cache_size = 1024*1024};
cache_t *cache = LRU_init(params, NULL);

// Process requests
request_t *req = new_request();
while (read_one_req(reader, req) == 0) {
    cache->get(cache, req);  // Returns true on hit
}
```

## CMake Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `ENABLE_TESTS` | ON | Enable test suite |
| `USE_HUGEPAGE` | ON | Transparent hugepage support |
| `SUPPORT_TTL` | OFF | Enable TTL support |
| `ENABLE_GLCACHE` | OFF | Group-learned cache (requires XGBoost) |
| `ENABLE_LRB` | OFF | LRB algorithm (requires LightGBM) |
| `LOG_LEVEL` | default | Logging: ERROR, WARN, INFO, DEBUG, VERBOSE |

## Code Style

- Google C/C++ style enforced via clang-format
- Strict compilation with `-Wall -Wextra -Werror`
- Pre-commit hooks available: `bash scripts/setup-hooks.sh`

## Key Algorithms

Recent state-of-the-art implementations:
- **S3-FIFO** (SOSP 2023) - `/libCacheSim/cache/eviction/S3FIFO.c`
- **SIEVE** (2023) - `/libCacheSim/cache/eviction/Sieve.c`
- **QDLP** (HotOS 2023) - `/libCacheSim/cache/eviction/QDLP.c`

Algorithms with `v0` suffix (e.g., `ARCv0`, `LRUv0`) are reference/original implementations.

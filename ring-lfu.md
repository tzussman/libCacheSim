# Ring-LFU: Technical Specification

---

## 1. Overview

Ring-LFU is a cache eviction policy that approximates LFU with time-based frequency decay. Items are organized into a ring buffer of B buckets, where logical bucket `k` holds items with effective frequency in `[2^k, 2^(k+1))`. Promotion between buckets is **probabilistic**: an access to an item in bucket `k` promotes it to bucket `k+1` with probability `1/2^k`. Aging advances the ring head, which implicitly halves every item's effective frequency. Aging is **O(1)** via list splicing.

There are no per-item frequency counters. The bucket position *is* the frequency estimate. Per-item eviction metadata is 1 byte.

### Properties

- O(1) access, insertion, eviction, and aging (all worst-case, not amortized).
- 1 byte of eviction metadata per item.
- Logarithmic frequency discrimination with LRU tiebreaking within each bucket.
- Frequency decays naturally over time without touching individual items.

---

## 2. Terminology

| Term | Definition |
|---|---|
| **B** | Number of ring buffer buckets. Determines max tracked frequency exponent. Typical: 16. |
| **Generation (G)** | Monotonically increasing epoch counter. Incremented once per aging step. |
| **Head (H)** | Physical index of logical bucket 0. `H = G mod B`. |
| **Logical offset** | `(physical − H + B) mod B`. Bucket at logical offset `k` holds items with effective frequency ≈ `2^k`. |
| **Zero-bucket** | Fixed bucket (outside the ring) for items with effective frequency 0. Evicted first. |
| **Physical bucket** | Index into the ring array, range `[0, B)`. |

---

## 3. Data Structures

### 3.1 Global State

```cpp
constexpr uint32_t B = 16;                        // Bucket count
constexpr uint32_t NULL_IDX = UINT32_MAX;
constexpr uint8_t  ZERO_BUCKET = 0xFF;             // Sentinel for zero-bucket membership

struct RingLFU {
    Bucket           ring[B];
    Bucket           zero_bucket;
    uint64_t         generation;
    uint32_t         nonempty_bitmap;               // Bit k set iff ring[k].count > 0
    HashTable<Key, uint32_t> index;                 // Key → item pool index
    ItemPool         pool;
    size_t           capacity;
    size_t           size;
};
```

### 3.2 Bucket

Doubly-linked list of items, ordered by recency (head = LRU, tail = MRU).

```cpp
struct Bucket {
    uint32_t head;          // LRU end (evict from here)
    uint32_t tail;          // MRU end (insert/promote here)
    uint32_t count;
    uint8_t  version;       // Incremented on each drain (see §6)
};
```

### 3.3 Item

Stored in a contiguous pre-allocated array. No heap allocation per item.

```cpp
struct Item {
    Key      key;
    Value    value;           // Or pointer to external storage

    uint32_t prev;            // DLL prev in bucket
    uint32_t next;            // DLL next in bucket

    uint32_t hash_next;       // Hash table chain

    // Eviction metadata: 1 byte total
    uint8_t  physical_bucket; // Which ring slot (or ZERO_BUCKET sentinel)
    uint8_t  placed_version;  // Bucket version when item was placed (see §6)
};
```

For high-throughput scanning, consider a struct-of-arrays layout: separate the hot fields (`physical_bucket`, `placed_version`, `prev`, `next`) from cold fields (`key`, `value`, `hash_next`) into parallel arrays.

### 3.4 Item Pool

```cpp
struct ItemPool {
    Item*    items;          // Length = capacity
    uint32_t free_head;      // Intrusive free list via `next` field
};
```

Free items are chained through `next`. No additional memory cost.

---

## 4. Core Operations

### 4.1 Helper: Logical Offset

```cpp
uint32_t LogicalOffset(uint8_t physical) {
    return (physical - (generation % B) + B) % B;
}

uint8_t PhysicalBucket(uint32_t logical_offset) {
    return (logical_offset + generation) % B;
}
```

### 4.2 Helper: DLL Operations

```cpp
void UnlinkFromBucket(Item& item, Bucket& bucket) {
    if (item.prev != NULL_IDX) pool[item.prev].next = item.next;
    else                       bucket.head = item.next;
    if (item.next != NULL_IDX) pool[item.next].prev = item.prev;
    else                       bucket.tail = item.prev;
    bucket.count--;
}

void PushTail(Item& item, Bucket& bucket, uint8_t phys, uint8_t ver) {
    item.prev = bucket.tail;
    item.next = NULL_IDX;
    if (bucket.tail != NULL_IDX) pool[bucket.tail].next = item.index;
    else                         bucket.head = item.index;
    bucket.tail = item.index;
    bucket.count++;
    item.physical_bucket = phys;
    item.placed_version = ver;
}
```

### 4.3 Helper: Resolve Bucket

Determines which bucket an item is actually in, accounting for stale bucket membership after a splice (see §6 for full explanation).

```cpp
Bucket& ResolveBucket(Item& item) {
    if (item.physical_bucket == ZERO_BUCKET)
        return zero_bucket;
    if (item.placed_version != ring[item.physical_bucket].version) {
        // Bucket was drained since this item was placed; it's in the zero-bucket.
        item.physical_bucket = ZERO_BUCKET;
        return zero_bucket;
    }
    return ring[item.physical_bucket];
}
```

### 4.4 Access (Cache Hit)

```cpp
Value* Access(Key key) {
    uint32_t idx = index.find(key);
    if (idx == NOT_FOUND) return nullptr;

    Item& item = pool[idx];
    Bucket& src = ResolveBucket(item);
    uint32_t k = (item.physical_bucket == ZERO_BUCKET)
                 ? 0
                 : LogicalOffset(item.physical_bucket);

    // Probabilistic promotion
    if (k < B - 1) {
        uint64_t r = prng.next();
        if (k == 0 || (r & ((1ULL << k) - 1)) == 0) {
            // Promote to bucket k+1
            uint8_t new_phys = PhysicalBucket(k + 1);
            UnlinkFromBucket(item, src);
            UpdateBitmap(src, item.physical_bucket);
            PushTail(item, ring[new_phys], new_phys, ring[new_phys].version);
            nonempty_bitmap |= (1u << new_phys);
        } else {
            // Stay in same bucket, move to MRU position
            UnlinkFromBucket(item, src);
            PushTail(item, src, item.physical_bucket, item.placed_version);
        }
    } else {
        // At max bucket, just refresh MRU position
        UnlinkFromBucket(item, src);
        PushTail(item, src, item.physical_bucket, item.placed_version);
    }

    return &item.value;
}
```

Note on `k == 0`: promotion probability is `1/2^0 = 1`, so items at logical offset 0 always promote on access. This is correct — a frequency-1 item that is accessed again should move to frequency ~2.

### 4.5 Insertion (Cache Miss)

```cpp
void Insert(Key key, Value value) {
    if (size >= capacity)
        Evict();

    uint32_t idx = pool.allocate();
    Item& item = pool[idx];
    item.key = key;
    item.value = value;

    // Insert at logical bucket 0 (frequency ≈ 1)
    uint8_t phys = PhysicalBucket(0);
    PushTail(item, ring[phys], phys, ring[phys].version);
    nonempty_bitmap |= (1u << phys);
    index.insert(key, idx);
    size++;
}
```

### 4.6 Eviction

```cpp
void Evict() {
    // Phase 1: evict from zero-bucket (LRU end)
    if (zero_bucket.count > 0) {
        RemoveItem(zero_bucket.head);
        return;
    }

    // Phase 2: find lowest non-empty ring bucket via bitmap
    uint32_t rotated = RotateRight(nonempty_bitmap, generation % B, B);
    assert(rotated != 0);  // size > 0 guarantees at least one non-empty bucket
    uint32_t offset = __builtin_ctz(rotated);
    uint8_t phys = (offset + generation) % B;

    RemoveItem(ring[phys].head);
}

void RemoveItem(uint32_t idx) {
    Item& item = pool[idx];
    Bucket& bucket = ResolveBucket(item);
    UnlinkFromBucket(item, bucket);
    UpdateBitmap(bucket, item.physical_bucket);
    index.remove(item.key);
    pool.free(idx);
    size--;
}
```

### 4.7 Deletion (Explicit Removal)

```cpp
void Delete(Key key) {
    uint32_t idx = index.find(key);
    if (idx == NOT_FOUND) return;
    RemoveItem(idx);
}
```

---

## 5. Aging

### 5.1 O(1) Splice

Aging drains the bucket at the current head position (logical offset 0, frequency ≈ 1) into the zero-bucket. After the generation increment, every item's logical offset decreases by 1 — items implicitly lose frequency without being touched.

```cpp
void Age() {
    uint8_t phys = generation % B;
    Bucket& bucket = ring[phys];

    // Bump version so stale items are detected (see §6)
    bucket.version++;

    // Splice entire bucket list into zero-bucket tail: O(1)
    if (bucket.count > 0) {
        if (zero_bucket.tail != NULL_IDX) {
            pool[zero_bucket.tail].next = bucket.head;
            pool[bucket.head].prev = zero_bucket.tail;
        } else {
            zero_bucket.head = bucket.head;
        }
        zero_bucket.tail = bucket.tail;
        zero_bucket.count += bucket.count;

        bucket.head = NULL_IDX;
        bucket.tail = NULL_IDX;
        bucket.count = 0;
        nonempty_bitmap &= ~(1u << phys);
    }

    generation++;
}
```

**Cost**: O(1). No individual items are visited.

### 5.2 Epoch Trigger Policy

Options (pick one or combine):

1. **Access-count**: every `capacity / B` operations. A full sweep of all buckets takes ~`capacity` operations.
2. **Adaptive**: trigger when `zero_bucket.count` drops below a threshold (e.g., `capacity / 4`), ensuring cheap eviction candidates are available.
3. **Wall-clock timer**: every T seconds (e.g., 60s). Appropriate for latency-sensitive systems.

---

## 6. Ghost Item Detection After Splice

### 6.1 The Problem

After a splice, moved items still carry their old `physical_bucket` value. On their next access, we need to detect that they're actually in the zero-bucket, not in the ring slot they think they're in. Otherwise we compute the wrong promotion probability and try to unlink from the wrong list.

### 6.2 Approach A: Per-Bucket Versioning (Recommended)

Each bucket has a `version` field, incremented on every drain. Each item stores the version of its bucket at placement time. On access, compare: if `item.placed_version != ring[item.physical_bucket].version`, the item is stale (in the zero-bucket).

**Metadata cost**: 1 byte per item (5 bits for `physical_bucket` with B ≤ 31, 3 bits for `placed_version`).

**Aliasing risk**: 3-bit version wraps every 8 drains of the same physical bucket. A physical bucket is drained once every B epochs, so aliasing requires `8 × B` epochs without access (128 epochs at B=16). Any item that cold is a correct eviction candidate regardless. The worst case on a false negative is one access with an incorrect promotion probability — self-correcting on the next access or eviction.

To eliminate aliasing entirely, widen the version to 8 bits (1 extra byte per bucket, negligible) and accept 2 bytes of per-item metadata.

### 6.3 Approach B: Sentinel-Node Circular DLLs

Each bucket (including the zero-bucket) uses a sentinel node. The sentinel's `next` is the head, its `prev` is the tail. Unlinking an item never requires knowing which bucket it belongs to:

```cpp
// Unlink is always:
pool[item.prev].next = item.next;
pool[item.next].prev = item.prev;
```

No head/tail edge cases, no bucket reference needed for pointer surgery.

**Trade-offs**: B+1 sentinel slots consumed from the item pool. Item count tracking requires a separate decrement — you still need to know which bucket's count to update, which brings back the ghost detection problem for count maintenance and bitmap updates. This approach is cleanest if you defer count/bitmap updates to access time, or use the version check specifically for count tracking while relying on sentinels for the unlink itself.

**Recommendation**: Approach A is simpler and self-contained. Use it unless you have a specific reason to prefer sentinels.

---

## 7. Admission Filter (Optional)

To resist scan pollution (bursts of unique keys evicting high-value items), gate insertion with a doorkeeper:

**Bloom filter approach**: Maintain a Bloom filter (10 bits/expected-key, 2-3 hash functions). On a cache miss for key K:
- If K is **not** in the Bloom filter: add K to the filter, do **not** insert into the cache.
- If K **is** in the filter: admit K to the cache.
- Clear the Bloom filter every epoch.

This ensures items must be seen at least twice before occupying a cache slot. Disable for workloads known to have no scan component.

---

## 8. Concurrency

### 8.1 Sharding

Partition the cache into S independent shards (S ≥ 2× hardware threads, power of two for bitwise routing):

```cpp
struct ShardedRingLFU {
    RingLFU  shards[S];
    size_t   route(Key k) { return hash(k) & (S - 1); }
};
```

Each shard has its own ring, zero-bucket, generation, and lock. No cross-shard contention.

### 8.2 Per-Shard Locking

Access mutates bucket lists, so it requires exclusive access. Use a per-shard mutex or spinlock. All operations (access, insert, evict, age) acquire the shard lock.

For reduced lock pressure:
- **Batched counter updates**: queue promotion decisions in a thread-local buffer, flush under lock periodically.
- **Thread-local L0 cache**: a small (4-16 entry) direct-mapped cache per thread, checked before the shard. L0 hits require no lock. Populate on shard hit, invalidate lazily.

### 8.3 Aging Under Concurrency

`Age()` acquires the shard lock like any other mutation. To avoid blocking access threads, trigger aging from within the normal operation path (e.g., the thread that performs the Nth access calls `Age()` while already holding the lock).

---

## 9. Additional Optimizations

### 9.1 Non-Empty Bitmap

A B-bit bitmap tracks which ring buckets are non-empty. Eviction finds the lowest non-empty logical bucket in O(1):

```cpp
uint32_t rotated = RotateRight(nonempty_bitmap, generation % B, B);
uint32_t offset = __builtin_ctz(rotated);
uint8_t  phys = (offset + generation) % B;
```

With B ≤ 32, this is a single `uint32_t` and all operations are single instructions.

### 9.2 PRNG Selection

The promotion test `(prng.next() & ((1 << k) - 1)) == 0` is the only per-access overhead beyond DLL manipulation. Use a fast inline PRNG — xorshift64 or SplitMix64. One PRNG state per shard (or per thread if using thread-local L0 caches). Seed from `rdrand` or `/dev/urandom`.

### 9.3 Ghost Entries for Adaptive Tuning

Maintain a small ring buffer of recently evicted key hashes (no values, ~4-8 bytes each). On a cache miss, check the ghost buffer. A high ghost-hit rate indicates the cache is evicting items that are still wanted — slow down aging by increasing the epoch interval. A low ghost-hit rate indicates aging is appropriately aggressive.

---

## 10. Tuning Parameters

| Parameter | Default | Effect |
|---|---|---|
| `B` (bucket count) | 16 | More buckets = finer frequency granularity. Diminishing returns past 32. |
| Epoch interval | `capacity / B` accesses | Shorter = more LRU-like (faster decay). Longer = more LFU-like. |
| Shard count `S` | 2 × num_threads | More shards = less contention, more memory fragmentation. |
| Admission filter | Bloom, 10 bits/key | Disable if no scan workload. |

---

## 11. Correctness Invariants

Assert these in debug builds:

1. Every item is in exactly one bucket (ring or zero) or on the free list. Never both.
2. `bucket.count` equals the actual list length.
3. `nonempty_bitmap` bit `k` is set iff `ring[k].count > 0`.
4. `index.find(item.key) == item.index` for every item in a bucket.
5. `size <= capacity`.
6. `generation` is monotonically increasing.

---

## 12. Worked Example

B = 4, capacity = 6.

**Initial state** (G=0, H=0):
```
Ring:   [0]  [1]  [2]  [3]    ZERO
Logic:   0    1    2    3
Items:   —    —    —    —      —
```

**Insert A, B, C** → logical bucket 0 = physical 0:
```
Ring:   [0:A→B→C]  [1]  [2]  [3]    ZERO
```

**Access A** (k=0, promote with p=1) → A moves to logical 1 = physical 1:
```
Ring:   [0:B→C]  [1:A]  [2]  [3]    ZERO
```

**Access A** (k=1, promote with p=1/2) → suppose PRNG succeeds → A moves to logical 2 = physical 2:
```
Ring:   [0:B→C]  [1]  [2:A]  [3]    ZERO
```

**Access A** (k=2, promote with p=1/4) → suppose PRNG fails → A stays, moves to MRU in same bucket:
```
Ring:   [0:B→C]  [1]  [2:A]  [3]    ZERO
```

**Age()** — G becomes 1, H becomes 1:
- Drain physical 0: splice B→C into zero-bucket. Bump `ring[0].version`.
```
Ring:   [0]  [1]  [2:A]  [3]    ZERO: B→C
Logic:   3    0    1     2
```
A is at physical 2, logical offset = (2−1+4)%4 = 1. Frequency halved: was ≈4, now ≈2. ✓

B and C have `physical_bucket=0, placed_version=old`. On next access, version check detects staleness → resolved as zero-bucket.

**Evict** → zero-bucket non-empty → evict B (LRU head of zero-bucket).

**Access C** (in zero-bucket, k=0, promote with p=1) → C moves to logical 0 = physical 1:
```
Ring:   [0]  [1:C]  [2:A]  [3]    ZERO: —
```
C is back in the game at frequency ≈1.

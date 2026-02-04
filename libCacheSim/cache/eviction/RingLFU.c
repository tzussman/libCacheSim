//
//  Ring-LFU: Approximate LFU with time-based frequency decay
//
//  Items are organized into a ring buffer of B buckets, where logical bucket k
//  holds items with effective frequency in [2^k, 2^(k+1)). Promotion between
//  buckets is probabilistic: an access to an item in bucket k promotes it to
//  bucket k+1 with probability 1/2^k. Aging advances the ring head, which
//  implicitly halves every item's effective frequency. Aging is O(1) via list
//  splicing.
//
//  RingLFU.c
//  libCacheSim
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"
#include "../../utils/include/mymath.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RINGLFU_NUM_BUCKETS 16
#define RINGLFU_ZERO_BUCKET 0xFF

typedef struct {
  cache_obj_t *head;  // LRU end (evict from here)
  cache_obj_t *tail;  // MRU end (insert/promote here)
  int64_t n_obj;
  int64_t n_byte;
  uint8_t version;  // Incremented on each drain
} RingLFU_bucket_t;

typedef struct RingLFU_params {
  RingLFU_bucket_t ring[RINGLFU_NUM_BUCKETS];
  RingLFU_bucket_t zero_bucket;
  uint64_t generation;
  uint32_t nonempty_bitmap;  // Bit k set iff ring[k].n_obj > 0
  int64_t age_interval;      // Age every N requests
  int64_t req_since_age;     // Requests since last age
} RingLFU_params_t;

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************

static void RingLFU_free(cache_t *cache);
static bool RingLFU_get(cache_t *cache, const request_t *req);
static cache_obj_t *RingLFU_find(cache_t *cache, const request_t *req,
                                 bool update_cache);
static cache_obj_t *RingLFU_insert(cache_t *cache, const request_t *req);
static cache_obj_t *RingLFU_to_evict(cache_t *cache, const request_t *req);
static void RingLFU_evict(cache_t *cache, const request_t *req);
static bool RingLFU_remove(cache_t *cache, obj_id_t obj_id);
static int64_t RingLFU_get_occupied_byte(const cache_t *cache);
static int64_t RingLFU_get_n_obj(const cache_t *cache);

/* internal functions */
static void RingLFU_parse_params(cache_t *cache,
                                 const char *cache_specific_params);
static inline uint32_t RingLFU_logical_offset(RingLFU_params_t *params,
                                              uint8_t physical);
static inline uint8_t RingLFU_physical_bucket(RingLFU_params_t *params,
                                              uint32_t logical_offset);
static inline RingLFU_bucket_t *RingLFU_resolve_bucket(RingLFU_params_t *params,
                                                       cache_obj_t *obj);
static void RingLFU_age(cache_t *cache);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief initialize a RingLFU cache
 *
 * @param ccache_params some common cache parameters
 * @param cache_specific_params RingLFU specific parameters:
 *   - age-interval: number of requests between aging (default: cache_size /
 * RINGLFU_NUM_BUCKETS bytes)
 */
cache_t *RingLFU_init(const common_cache_params_t ccache_params,
                      const char *cache_specific_params) {
  cache_t *cache =
      cache_struct_init("RingLFU", ccache_params, cache_specific_params);
  cache->cache_init = RingLFU_init;
  cache->cache_free = RingLFU_free;
  cache->get = RingLFU_get;
  cache->find = RingLFU_find;
  cache->insert = RingLFU_insert;
  cache->evict = RingLFU_evict;
  cache->remove = RingLFU_remove;
  cache->to_evict = RingLFU_to_evict;
  cache->get_occupied_byte = RingLFU_get_occupied_byte;
  cache->get_n_obj = RingLFU_get_n_obj;

  // 2 bytes per object metadata: physical_bucket + placed_version
  if (ccache_params.consider_obj_metadata) {
    cache->obj_md_size = 2;
  } else {
    cache->obj_md_size = 0;
  }

  RingLFU_params_t *params = my_malloc(RingLFU_params_t);
  memset(params, 0, sizeof(RingLFU_params_t));
  cache->eviction_params = params;

  // Initialize all ring buckets
  for (int i = 0; i < RINGLFU_NUM_BUCKETS; i++) {
    params->ring[i].head = NULL;
    params->ring[i].tail = NULL;
    params->ring[i].n_obj = 0;
    params->ring[i].n_byte = 0;
    params->ring[i].version = 0;
  }

  // Initialize zero bucket
  params->zero_bucket.head = NULL;
  params->zero_bucket.tail = NULL;
  params->zero_bucket.n_obj = 0;
  params->zero_bucket.n_byte = 0;
  params->zero_bucket.version = 0;

  params->generation = 0;
  params->nonempty_bitmap = 0;

  // Default age interval: roughly capacity / B requests
  // This means a full sweep of all buckets takes ~capacity requests
  params->age_interval = (int64_t)(ccache_params.cache_size / 1024 /
                                   RINGLFU_NUM_BUCKETS);  // Assume 1KB avg size
  if (params->age_interval < 100) {
    params->age_interval = 100;  // Minimum interval
  }
  params->req_since_age = 0;

  if (cache_specific_params != NULL) {
    RingLFU_parse_params(cache, cache_specific_params);
  }

  return cache;
}

/**
 * free resources used by this cache
 *
 * @param cache
 */
static void RingLFU_free(cache_t *cache) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);
  my_free(sizeof(RingLFU_params_t), params);
  cache_struct_free(cache);
}

/**
 * @brief this function is the user facing API
 * it performs the following logic
 *
 * ```
 * if obj in cache:
 *    update_metadata
 *    return true
 * else:
 *    if cache does not have enough space:
 *        evict until it has space to insert
 *    insert the object
 *    return false
 * ```
 *
 * @param cache
 * @param req
 * @return true if cache hit, false if cache miss
 */
static bool RingLFU_get(cache_t *cache, const request_t *req) {
  return cache_get_base(cache, req);
}

// ***********************************************************************
// ****                                                               ****
// ****       developer facing APIs (used by cache developer)         ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief find an object in the cache
 *
 * @param cache
 * @param req
 * @param update_cache whether to update the cache,
 *  if true, the object is promoted probabilistically
 * @return the object or NULL if not found
 */
static cache_obj_t *RingLFU_find(cache_t *cache, const request_t *req,
                                 bool update_cache) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);
  cache_obj_t *obj = cache_find_base(cache, req, update_cache);

  if (obj == NULL) {
    return NULL;
  }

  if (!update_cache) {
    return obj;
  }

  // Check for aging
  params->req_since_age++;
  if (params->req_since_age >= params->age_interval) {
    RingLFU_age(cache);
    params->req_since_age = 0;
  }

  // Resolve which bucket the object is actually in
  RingLFU_bucket_t *src_bucket = RingLFU_resolve_bucket(params, obj);
  uint32_t k;  // logical offset

  if (obj->RingLFU.physical_bucket == RINGLFU_ZERO_BUCKET) {
    k = 0;
  } else {
    k = RingLFU_logical_offset(params, obj->RingLFU.physical_bucket);
  }

  // Probabilistic promotion
  bool should_promote = false;
  if (k < RINGLFU_NUM_BUCKETS - 1) {
    if (k == 0) {
      // Items at logical offset 0 always promote (probability = 1/2^0 = 1)
      should_promote = true;
    } else {
      // Probability 1/2^k: check if lower k bits of random are all zero
      uint64_t r = next_rand();
      uint64_t mask = (1ULL << k) - 1;
      should_promote = ((r & mask) == 0);
    }
  }

  if (should_promote) {
    // Remove from source bucket
    remove_obj_from_list(&src_bucket->head, &src_bucket->tail, obj);
    src_bucket->n_obj--;
    src_bucket->n_byte -= obj->obj_size + cache->obj_md_size;

    // Update bitmap if source bucket is now empty
    if (obj->RingLFU.physical_bucket != RINGLFU_ZERO_BUCKET &&
        src_bucket->n_obj == 0) {
      params->nonempty_bitmap &= ~(1u << obj->RingLFU.physical_bucket);
    }

    // Promote to bucket k+1
    uint8_t new_phys = RingLFU_physical_bucket(params, k + 1);
    RingLFU_bucket_t *dst_bucket = &params->ring[new_phys];

    append_obj_to_tail(&dst_bucket->head, &dst_bucket->tail, obj);
    dst_bucket->n_obj++;
    dst_bucket->n_byte += obj->obj_size + cache->obj_md_size;

    obj->RingLFU.physical_bucket = new_phys;
    obj->RingLFU.placed_version = dst_bucket->version;

    params->nonempty_bitmap |= (1u << new_phys);
  }
  // No movement on non-promoting access - eviction is arbitrary within bucket

  return obj;
}

/**
 * @brief insert an object into the cache,
 * update the hash table and cache metadata
 * this function assumes the cache has enough space
 * eviction should be performed before calling this function
 *
 * @param cache
 * @param req
 * @return the inserted object
 */
static cache_obj_t *RingLFU_insert(cache_t *cache, const request_t *req) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);

  cache_obj_t *obj = cache_insert_base(cache, req);

  // Insert at logical bucket 0 (frequency ≈ 1)
  uint8_t phys = RingLFU_physical_bucket(params, 0);
  RingLFU_bucket_t *bucket = &params->ring[phys];

  append_obj_to_tail(&bucket->head, &bucket->tail, obj);
  bucket->n_obj++;
  bucket->n_byte += req->obj_size + cache->obj_md_size;

  obj->RingLFU.physical_bucket = phys;
  obj->RingLFU.placed_version = bucket->version;

  params->nonempty_bitmap |= (1u << phys);

  return obj;
}

/**
 * @brief find the object to be evicted
 * this function does not actually evict the object or update metadata
 *
 * @param cache the cache
 * @return the object to be evicted
 */
static cache_obj_t *RingLFU_to_evict(cache_t *cache, const request_t *req) {
  UNUSED(req);
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);

  // Phase 1: evict from zero-bucket first (LRU end)
  if (params->zero_bucket.n_obj > 0) {
    return params->zero_bucket.head;
  }

  // Phase 2: find lowest non-empty ring bucket via bitmap
  if (params->nonempty_bitmap == 0) {
    return NULL;  // No objects in cache
  }

  // Rotate bitmap to align logical bucket 0 with bit 0
  uint32_t head_pos = params->generation % RINGLFU_NUM_BUCKETS;
  uint32_t rotated = (params->nonempty_bitmap >> head_pos) |
                     (params->nonempty_bitmap << (RINGLFU_NUM_BUCKETS - head_pos));
  rotated &= ((1u << RINGLFU_NUM_BUCKETS) - 1);  // Mask to B bits

  // Find lowest set bit (lowest logical bucket with items)
  uint32_t offset = (uint32_t)__builtin_ctz(rotated);
  uint8_t phys = (uint8_t)((offset + head_pos) % RINGLFU_NUM_BUCKETS);

  return params->ring[phys].head;  // LRU end of this bucket
}

/**
 * @brief evict an object from the cache
 * it needs to call cache_evict_base before returning
 * which updates some metadata such as n_obj, occupied size, and hash table
 *
 * @param cache
 * @param req not used
 */
static void RingLFU_evict(cache_t *cache, const request_t *req) {
  UNUSED(req);
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);

  cache_obj_t *obj = RingLFU_to_evict(cache, req);
  if (obj == NULL) {
    return;
  }

  // Resolve the actual bucket (handles stale items)
  RingLFU_bucket_t *bucket = RingLFU_resolve_bucket(params, obj);

  remove_obj_from_list(&bucket->head, &bucket->tail, obj);
  bucket->n_obj--;
  bucket->n_byte -= obj->obj_size + cache->obj_md_size;

  // Update bitmap if this was a ring bucket and is now empty
  if (obj->RingLFU.physical_bucket != RINGLFU_ZERO_BUCKET &&
      bucket->n_obj == 0) {
    params->nonempty_bitmap &= ~(1u << obj->RingLFU.physical_bucket);
  }

  cache_evict_base(cache, obj, true);
}

/**
 * @brief remove an object from the cache
 * this is different from cache_evict because it is used to for user trigger
 * remove, and eviction is used by the cache to make space for new objects
 *
 * @param cache
 * @param obj_id
 * @return true if the object is removed, false if the object is not in the
 * cache
 */
static bool RingLFU_remove(cache_t *cache, obj_id_t obj_id) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);
  cache_obj_t *obj = hashtable_find_obj_id(cache->hashtable, obj_id);

  if (obj == NULL) {
    return false;
  }

  // Resolve the actual bucket
  RingLFU_bucket_t *bucket = RingLFU_resolve_bucket(params, obj);

  remove_obj_from_list(&bucket->head, &bucket->tail, obj);
  bucket->n_obj--;
  bucket->n_byte -= obj->obj_size + cache->obj_md_size;

  // Update bitmap if this was a ring bucket and is now empty
  if (obj->RingLFU.physical_bucket != RINGLFU_ZERO_BUCKET &&
      bucket->n_obj == 0) {
    params->nonempty_bitmap &= ~(1u << obj->RingLFU.physical_bucket);
  }

  cache_remove_obj_base(cache, obj, true);
  return true;
}

// ***********************************************************************
// ****                                                               ****
// ****                       internal functions                      ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief get the logical offset of a physical bucket
 * Logical offset k means items have effective frequency ≈ 2^k
 */
static inline uint32_t RingLFU_logical_offset(RingLFU_params_t *params,
                                              uint8_t physical) {
  uint32_t head = params->generation % RINGLFU_NUM_BUCKETS;
  return (physical - head + RINGLFU_NUM_BUCKETS) % RINGLFU_NUM_BUCKETS;
}

/**
 * @brief get the physical bucket index for a logical offset
 */
static inline uint8_t RingLFU_physical_bucket(RingLFU_params_t *params,
                                              uint32_t logical_offset) {
  return (uint8_t)((logical_offset + params->generation) % RINGLFU_NUM_BUCKETS);
}

/**
 * @brief resolve which bucket an item is actually in
 * Handles detection of stale items that were spliced to zero-bucket
 */
static inline RingLFU_bucket_t *RingLFU_resolve_bucket(RingLFU_params_t *params,
                                                       cache_obj_t *obj) {
  if (obj->RingLFU.physical_bucket == RINGLFU_ZERO_BUCKET) {
    return &params->zero_bucket;
  }

  // Check if the item is stale (bucket was drained since placement)
  if (obj->RingLFU.placed_version !=
      params->ring[obj->RingLFU.physical_bucket].version) {
    // Bucket was drained; item is actually in zero-bucket
    obj->RingLFU.physical_bucket = RINGLFU_ZERO_BUCKET;
    return &params->zero_bucket;
  }

  return &params->ring[obj->RingLFU.physical_bucket];
}

/**
 * @brief age the cache by draining the bucket at the current head position
 * into the zero-bucket. This implicitly halves every item's effective
 * frequency. O(1) operation via list splicing.
 */
static void RingLFU_age(cache_t *cache) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);

  uint8_t phys = (uint8_t)(params->generation % RINGLFU_NUM_BUCKETS);
  RingLFU_bucket_t *bucket = &params->ring[phys];

  // Bump version so stale items are detected on access
  bucket->version++;

  // Splice entire bucket list into zero-bucket tail: O(1)
  if (bucket->n_obj > 0) {
    if (params->zero_bucket.tail != NULL) {
      // Link zero-bucket tail to bucket head
      params->zero_bucket.tail->queue.next = bucket->head;
      bucket->head->queue.prev = params->zero_bucket.tail;
    } else {
      // Zero-bucket was empty
      params->zero_bucket.head = bucket->head;
    }
    params->zero_bucket.tail = bucket->tail;
    params->zero_bucket.n_obj += bucket->n_obj;
    params->zero_bucket.n_byte += bucket->n_byte;

    // Clear the ring bucket
    bucket->head = NULL;
    bucket->tail = NULL;
    bucket->n_obj = 0;
    bucket->n_byte = 0;
    params->nonempty_bitmap &= ~(1u << phys);
  }

  params->generation++;
}

/**
 * @brief get total occupied bytes across all buckets
 */
static int64_t RingLFU_get_occupied_byte(const cache_t *cache) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);

  int64_t total = params->zero_bucket.n_byte;
  for (int i = 0; i < RINGLFU_NUM_BUCKETS; i++) {
    total += params->ring[i].n_byte;
  }
  return total;
}

/**
 * @brief get total number of objects across all buckets
 */
static int64_t RingLFU_get_n_obj(const cache_t *cache) {
  RingLFU_params_t *params = (RingLFU_params_t *)(cache->eviction_params);

  int64_t total = params->zero_bucket.n_obj;
  for (int i = 0; i < RINGLFU_NUM_BUCKETS; i++) {
    total += params->ring[i].n_obj;
  }
  return total;
}

// ***********************************************************************
// ****                                                               ****
// ****                  parameter set up functions                   ****
// ****                                                               ****
// ***********************************************************************

static const char *RingLFU_current_params(RingLFU_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "age-interval=%ld", (long)params->age_interval);
  return params_str;
}

static void RingLFU_parse_params(cache_t *cache,
                                 const char *cache_specific_params) {
  RingLFU_params_t *params = (RingLFU_params_t *)cache->eviction_params;
  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;
  char *end;

  while (params_str != NULL && params_str[0] != '\0') {
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // Skip whitespace
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    if (strcasecmp(key, "age-interval") == 0) {
      params->age_interval = (int64_t)strtol(value, &end, 0);
      if (strlen(end) > 2) {
        ERROR("param parsing error, find string \"%s\" after number\n", end);
      }
    } else if (strcasecmp(key, "print") == 0) {
      printf("current parameters: %s\n", RingLFU_current_params(params));
      exit(0);
    } else {
      ERROR("%s does not have parameter %s\n", cache->cache_name, key);
      exit(1);
    }
  }
  free(old_params_str);
}

#ifdef __cplusplus
}
#endif

#pragma once
#include <math.h>
#include <stdint.h>

// Operates on median water heights, in cm. No Arduino dependency so recorded
// sensor traces can exercise the same filter on a desktop.
class LevelFilter {
 public:
  float height = 0;
  float flow = 0;
  // Fast movement hint for the dashboard animation: +1 filling, -1 draining,
  // 0 settled. Confirmed from consecutive same-direction steps, so sensor
  // noise cannot fake it, and it reacts long before `flow` has enough history.
  int8_t trend = 0;
  void reset() {
    ready = false; count = 0; direction = 0; confirmations = 0; flow = 0;
    trend = 0; tCount = 0; trendHold = 0;
  }

  void update(float rawHeight, uint32_t now, float area) {
    if (!ready) {
      ready = true;
      smooth = height = rawHeight;
      recent[0] = recent[1] = recent[2] = rawHeight;
      recentIndex = 0;
      lastChange = now;
      count = 0;
    }
    recent[recentIndex] = rawHeight;
    recentIndex = (recentIndex + 1) % 3;
    float a = recent[0], b = recent[1], c = recent[2];
    float median = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
    smooth += 0.35f * (median - smooth);

    // The sensor quantises to ~0.4 cm, so consecutive differences are dominated
    // by that step and their sign is meaningless. A least-squares slope over a
    // short window averages the quantisation out and gives a direction that
    // still water cannot fake.
    if (tCount == TREND_N) {
      for (int i = 1; i < TREND_N; ++i) { tVal[i - 1] = tVal[i]; tMs[i - 1] = tMs[i]; }
      --tCount;
    }
    tVal[tCount] = smooth;
    tMs[tCount] = now;
    ++tCount;
    if (tCount >= TREND_MIN_N) {
      float sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (int i = 0; i < tCount; ++i) {
        const float x = (tMs[i] - tMs[0]) / 1000.0f;   // seconds
        sx += x; sy += tVal[i]; sxx += x * x; sxy += x * tVal[i];
      }
      const float den = tCount * sxx - sx * sx;
      if (den > 1e-4f) {
        const float slope = (tCount * sxy - sx * sy) / den;   // cm per second
        if (slope >= TREND_SLOPE_CMS)       { trend = 1;  trendHold = now; }
        else if (slope <= -TREND_SLOPE_CMS) { trend = -1; trendHold = now; }
        else if (now - trendHold > TREND_RELEASE_MS) { trend = 0; }
      }
    }

    float delta = smooth - height;
    int nextDirection = delta > 0.15f ? 1 : delta < -0.15f ? -1 : 0;
    confirmations = nextDirection && nextDirection == direction ? confirmations + 1 : (nextDirection ? 1 : 0);
    direction = nextDirection;
    if (confirmations >= 2) {
      height = roundf(smooth * 10.0f) / 10.0f;
      lastChange = now;
      confirmations = 0;
    }

    // Five seconds of accepted levels, using elapsed time rather than assuming
    // a constant sensor interval. Reset after quiet periods to discard old flow.
    while (count && now - times[0] > 5000) removeOldest();
    if (count == 16) removeOldest();
    times[count] = now;
    heights[count++] = height;
    flow = 0;
    if (now - lastChange >= 2500) {
      count = 1;
      times[0] = now;
      heights[0] = height;
    } else if (count >= 3 && now - times[0] >= 1500) {
      float change = height - heights[0];
      if (fabsf(change) >= 0.15f) {
        flow = change * area * 60.0f / float(now - times[0]);
      }
    }
  }

 private:
  // Tuned for a 250 ms sample interval.
  static constexpr int      TREND_N           = 8;      // 2.0 s regression window
  static constexpr int      TREND_MIN_N       = 5;      // decide from 1.25 s onward
  static constexpr float    TREND_SLOPE_CMS   = 0.05f;  // cm/s that counts as real movement
  static constexpr uint32_t TREND_RELEASE_MS  = 1200;   // stop animating 1.2 s after movement ends
  float tVal[TREND_N] = {};
  uint32_t tMs[TREND_N] = {};
  int tCount = 0;
  uint32_t trendHold = 0;
  bool ready = false;
  float smooth = 0;
  float recent[3] = {};
  int recentIndex = 0;
  uint32_t lastChange = 0;
  uint32_t times[16] = {};
  float heights[16] = {};
  int count = 0, direction = 0, confirmations = 0;
  void removeOldest() {
    for (int i = 1; i < count; ++i) { times[i - 1] = times[i]; heights[i - 1] = heights[i]; }
    --count;
  }
};
